#pragma once
/* BLE 連線 (NimBLE):讓手機 app 取代 USB 線。
 *
 * GATT 一個服務、四個特徵值 (手機端 app/lib/ble.dart 的 UUID 必須一致):
 *   CMD    7a5e0002  Write        ASCII 指令,和序列埠同一套:TEST REC STOP DUMP CLEAR
 *   STATUS 7a5e0003  Notify 1 Hz  ASCII 一行:S,ms,score,thr,state,nRR,har,rec,test,hr,phone,moving,phase,left,idx,n,worn,quality
 *   STREAM 7a5e0004  Notify       二進位:'P' 濾波後 PPG 32 Hz ×10 int16 (畫波形用);
 *                                 'R' 原始 PPG 100 Hz (seq, ms, 25×(u32 綠, u32 IR));'I' IMU 20 Hz (seq, ms, 10×3 int16 ×1000)
 *   FILE   7a5e0005  Notify       檔案傳輸:DUMP 回 "L,<bytes>\n";GET,<off>,<len> 回 [uint32 off][資料] 多包
 *
 * 指令由 NimBLE 主機任務收進佇列,實際執行在 loop() (core 1),避免與 LittleFS、錄音環形緩衝競爭。
 * 檔案傳輸在獨立任務 (core 0) 送,不阻塞畫面。 */
#include <NimBLEDevice.h>
#include <LittleFS.h>

namespace ble {

static const char *UUID_SVC    = "7a5e0001-8f2b-4c1d-9e3a-0c1f2a3b4c5d";
static const char *UUID_CMD    = "7a5e0002-8f2b-4c1d-9e3a-0c1f2a3b4c5d";
static const char *UUID_STATUS = "7a5e0003-8f2b-4c1d-9e3a-0c1f2a3b4c5d";
static const char *UUID_STREAM = "7a5e0004-8f2b-4c1d-9e3a-0c1f2a3b4c5d";
static const char *UUID_FILE   = "7a5e0005-8f2b-4c1d-9e3a-0c1f2a3b4c5d";

static NimBLEServer         *server = nullptr;
static NimBLECharacteristic *chStatus = nullptr, *chStream = nullptr, *chFile = nullptr;
static volatile bool connected = false;
static volatile uint32_t connects = 0, txBytes = 0, streamPkts = 0, fastMs = 0;   // 2026-09-21 耗電歸因用:連線次數、傳檔位元組、串流封包數、快速連線間隔累計 ms (寫進 /bat.csv)
static uint32_t fastSince = 0;
static volatile bool streamOn  = false;      // 手機訂閱 STREAM 才送波形
static volatile bool fileBusy  = false;
static bool     fastConn = false; static uint32_t fastUntil = 0;
static bool     background = false;   // BG,1:app 在背景 → 狀態列 5 s 一筆、連線間隔 200–400 ms (2026-09-22)

// 指令佇列 (NimBLE 任務 → loop):環形 8 筆。2026-09-15 前只有一格,手機連上時 REC 與 TIME 背靠背寫入會丟掉一筆
// (實測三輪耗電測試快取都沒有 rec 標記),改成佇列。
static constexpr int CMD_Q = 8;
static char             cmdQ[CMD_Q][32];
static volatile uint8_t cmdHead = 0, cmdTail = 0;
static volatile uint8_t cmdDropped = 0;

// PPG 串流批次:32 Hz,16 筆一包 (0.5 s)
static int16_t  ppgPkt[1 + 16 * 1]; static int ppgN = 0; static int ppgDecim = 0;
// IMU 串流批次:20 Hz,10 筆 ×3 軸一包 (0.5 s):'I', seq(u8), ms(u32 LE, 第一筆時間), 10×3 int16
static uint8_t  imuPkt[6 + 10 * 3 * 2]; static int imuN = 0; static uint8_t imuSeq = 0;
// 原始 PPG 100 Hz,25 筆一包 (0.25 s):'R', seq(u8), ms(u32 LE, 第一筆時間), 25×(u32 綠光, u32 IR)
static uint8_t  rawPkt[6 + 25 * 8]; static int rawN = 0; static uint8_t rawSeq = 0;

static inline void put32(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }

static inline int16_t clip16(float v)
{
    return v > 32767.f ? 32767 : v < -32768.f ? -32768 : (int16_t)lrintf(v);
}

struct ServerCb : NimBLEServerCallbacks {
    void onConnect(NimBLEServer *, NimBLEConnInfo &info) override
    {
        connected = true;
        // 平常 50–100 ms 間隔省電;傳檔時 setFast(true) 暫時縮到 7.5–15 ms;沒有串流時再加 slave latency 4 (見 applyConn)
        server->updateConnParams(info.getConnHandle(), 40, 80, 4, 400);
        Serial.println("# BLE connected"); connects++;
    }
    void onDisconnect(NimBLEServer *, NimBLEConnInfo &, int reason) override
    {
        connected = false; streamOn = false; fastConn = false; background = false;
        Serial.printf("# BLE disconnected (%d), advertising\n", reason);
        NimBLEDevice::startAdvertising();
    }
    void onMTUChange(uint16_t mtu, NimBLEConnInfo &) override { Serial.printf("# BLE mtu=%u\n", mtu); }
};

struct CmdCb : NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &) override
    {
        uint8_t next = (uint8_t)((cmdHead + 1) % CMD_Q);
        if (next == cmdTail) { cmdDropped++; return; }   // 佇列滿 (loop 太久沒取)
        char *b = cmdQ[cmdHead];
        NimBLEAttValue v = c->getValue();
        size_t n = v.length() < sizeof(cmdQ[0]) - 1 ? v.length() : sizeof(cmdQ[0]) - 1;
        memcpy(b, v.data(), n);
        while (n && (b[n - 1] == '\n' || b[n - 1] == '\r')) n--;
        b[n] = 0;
        cmdHead = next;
    }
};

static void applyConn();
struct StreamCb : NimBLECharacteristicCallbacks {
    void onSubscribe(NimBLECharacteristic *, NimBLEConnInfo &, uint16_t sub) override
    {
        streamOn = sub != 0; ppgN = 0; imuN = 0; rawN = 0;
        Serial.printf("# BLE stream %s\n", streamOn ? "on" : "off");
        applyConn();
    }
};

static void begin(const char *name)
{
    NimBLEDevice::init(name);
    NimBLEDevice::setMTU(247);
    NimBLEDevice::setPower(ESP_PWR_LVL_P6);
    server = NimBLEDevice::createServer();
    server->setCallbacks(new ServerCb());
    NimBLEService *svc = server->createService(UUID_SVC);
    NimBLECharacteristic *cmd = svc->createCharacteristic(
        UUID_CMD, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    cmd->setCallbacks(new CmdCb());
    chStatus = svc->createCharacteristic(UUID_STATUS, NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);
    chStream = svc->createCharacteristic(UUID_STREAM, NIMBLE_PROPERTY::NOTIFY);
    chStream->setCallbacks(new StreamCb());
    chFile   = svc->createCharacteristic(UUID_FILE,   NIMBLE_PROPERTY::NOTIFY);
    svc->start();
    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    adv->setName(name);
    adv->addServiceUUID(UUID_SVC);
    adv->enableScanResponse(true);
    adv->setMinInterval(400); adv->setMaxInterval(800);   // 250–500 ms 廣播間隔 (省電;手機幾秒內仍找得到)
    adv->start();
    Serial.printf("# BLE advertising as %s\n", name);
}

/* 連線參數 (2026-09-11 省電):
 *   傳檔 (fastConn)      7.5–15 ms,latency 0
 *   串流中 (手機訂閱波形) 50–100 ms,latency 0
 *   只送狀態列           50–100 ms,latency 4 → 手錶沒東西送時可以連跳 4 個連線事件,無線電最少每 0.5 s 醒一次
 * 手機端只在儀表板可見 / 校正中 / 使用者開「持續傳原始波形」時才訂閱串流。 */
static void applyConn()
{
    if (!connected) return;
    uint16_t h = server->getPeerInfo(0).getConnHandle();
    if (fastConn)            server->updateConnParams(h, 6, 12, 0, 400);      // 傳檔 7.5–15 ms
    else if (streamOn)       server->updateConnParams(h, 40, 80, 0, 400);     // 串流 50–100 ms
    else if (background)     server->updateConnParams(h, 160, 320, 4, 600);   // 背景 200–400 ms + latency 4 → 無線電最少每 2 s 醒;監督逾時 6 s (規格要求 > 2×(1+latency)×間隔 = 4 s)
    else                     server->updateConnParams(h, 40, 80, 4, 400);     // 前景只送狀態列
}
static void setBackground(bool on)
{
    if (background == on) return;
    background = on;
    Serial.printf("# app %s\n", on ? "background: status 5 s, conn 200-400 ms" : "foreground: status 1 s, conn 50-100 ms");
    applyConn();
}
/* 連線間隔切換:傳檔期間 7.5–15 ms,結束 5 秒後回到平常設定 */
static void setFast(bool on)
{
    if (!connected || fastConn == on) return;
    if (on) fastSince = millis(); else if (fastSince) { fastMs += millis() - fastSince; fastSince = 0; }
    fastConn = on;
    applyConn();
}
static void tick(uint32_t now) { if (fastConn && (int32_t)(now - fastUntil) > 0) setFast(false); }

/* loop() 每圈呼叫:有指令就複製出來並回傳 true。 */
static bool takeCommand(char *out, size_t cap)
{
    if (cmdTail == cmdHead) return false;
    strncpy(out, cmdQ[cmdTail], cap - 1); out[cap - 1] = 0;
    cmdTail = (uint8_t)((cmdTail + 1) % CMD_Q);
    return true;
}

static void sendStatus(const char *line)
{
    if (!connected || !chStatus) return;
    chStatus->setValue((const uint8_t *)line, strlen(line));
    chStatus->notify();
}

/* 處理鏈輸出 (64 Hz) 餵進來,降到 32 Hz、×10 打包 */
static void pushPpg(float filt)
{
    if (!streamOn) return;
    if ((ppgDecim++ & 1) == 0) return;
    ppgPkt[1 + ppgN++] = clip16(filt * 10.f);
    if (ppgN == 16) {
        ppgPkt[0] = 'P';
        chStream->setValue((const uint8_t *)ppgPkt, sizeof(ppgPkt));
        chStream->notify(); streamPkts++;
        ppgN = 0;
    }
}

static void pushImu(float ax, float ay, float az, uint32_t ms)
{
    if (!streamOn) return;
    if (imuN == 0) { imuPkt[0] = 'I'; imuPkt[1] = imuSeq++; put32(imuPkt + 2, ms); }
    int16_t v[3] = {clip16(ax * 1000.f), clip16(ay * 1000.f), clip16(az * 1000.f)};
    memcpy(imuPkt + 6 + imuN * 6, v, 6);
    if (++imuN == 10) {
        chStream->setValue(imuPkt, sizeof(imuPkt));
        chStream->notify(); streamPkts++;
        imuN = 0;
    }
}

/* 原始綠光讀值 (100 Hz) 直接送,手機端存檔用;時間戳是手錶的 millis,掉包由序號看出 */
static void pushRaw(uint32_t raw, uint32_t ir, uint32_t ms)
{
    if (!streamOn) return;
    if (rawN == 0) { rawPkt[0] = 'R'; rawPkt[1] = rawSeq++; put32(rawPkt + 2, ms); }
    put32(rawPkt + 6 + rawN * 8, raw); put32(rawPkt + 10 + rawN * 8, ir);
    if (++rawN == 25) {
        chStream->setValue(rawPkt, sizeof(rawPkt));
        chStream->notify(); streamPkts++;
        rawN = 0;
    }
}

/* /log.csv 傳輸:手機主導的拉取協定 (notify 沒有回執,手機緩衝一滿封包就掉,推送式會卡死)。
 *   DUMP            → 回一包 "L,<bytes>\n";SYNC,<after>[,1] → "L,<bytes>,<lo>,<hi>\n" 只含序號 > after 的完成檔 (,1 先把目前檔收尾)
 *   GET,<off>,<len> → 從 off 起送 len 個位元組,每包 [uint32 off LE][資料],包大小 = MTU-3
 * 手機比對收到的偏移量,缺的範圍再要一次。每個 GET 在獨立任務 (core 0) 送,不阻塞畫面。 */
static uint32_t getOff = 0, getLen = 0;

// 檔案內容由 main.cpp 的快取環提供 (多個 30 秒檔串接)
uint32_t fileTotal();                       // 定義在 main.cpp
uint32_t fileSnapshot();                    // DUMP:重拍快照並回總量
uint32_t fileSnapshotRange(uint32_t after, bool rotate, uint32_t *lo, uint32_t *hi);   // SYNC:只拍序號 > after 的完成檔
size_t   fileRead(uint32_t off, uint8_t *buf, size_t len);
static void sendHeader()
{
    char hdr[24]; int hn = snprintf(hdr, sizeof(hdr), "L,%lu\n", (unsigned long)fileSnapshot());
    chFile->setValue((const uint8_t *)hdr, hn); chFile->notify();
}

static void getTask(void *)
{
    uint32_t total = fileTotal();
    if (connected) {
        uint16_t mtu = server->getPeerMTU(server->getPeerInfo(0).getConnHandle());
        size_t chunk = (mtu > 23 ? mtu - 3 : 20); if (chunk > 244) chunk = 244;
        static uint8_t buf[244];
        uint32_t off = getOff, end = getOff + getLen;
        if (end > total) end = total;
        while (off < end && connected) {
            size_t n = fileRead(off, buf + 4, (end - off < chunk - 4) ? end - off : chunk - 4);
            if (n == 0) break;
            buf[0] = off & 0xFF; buf[1] = (off >> 8) & 0xFF; buf[2] = (off >> 16) & 0xFF; buf[3] = (off >> 24) & 0xFF;
            chFile->setValue(buf, n + 4);
            int tries = 0;
            while (!chFile->notify() && connected && tries++ < 400) vTaskDelay(pdMS_TO_TICKS(5));
            if (tries >= 400) break;               // 2 秒送不出去就放棄這輪,手機會再要
            off += n; txBytes += n;
            vTaskDelay(pdMS_TO_TICKS(2));          // 讓主機任務有空把包送出去
        }
    }
    fileBusy = false;
    vTaskDelete(nullptr);
}

static void sendFile()            // DUMP:只回檔頭
{
    if (!connected) return;
    setFast(true); fastUntil = millis() + 5000;
    sendHeader();
}

static void sendSync(uint32_t after, bool rotate)   // SYNC:回 "L,<bytes>,<lo seq>,<hi seq>",之後手機照樣 GET
{
    if (!connected || fileBusy) return;
    setFast(true); fastUntil = millis() + 5000;
    uint32_t lo = 0, hi = 0, total = fileSnapshotRange(after, rotate, &lo, &hi);
    char hdr[48]; int hn = snprintf(hdr, sizeof(hdr), "L,%lu,%lu,%lu\n", (unsigned long)total, (unsigned long)lo, (unsigned long)hi);
    chFile->setValue((const uint8_t *)hdr, hn); chFile->notify();
    Serial.printf("# BLE sync after=%lu rotate=%d -> %lu bytes seq %lu..%lu\n", (unsigned long)after, rotate ? 1 : 0, (unsigned long)total, (unsigned long)lo, (unsigned long)hi);
}

static void sendRange(uint32_t off, uint32_t len)
{
    if (fileBusy || !connected) return;
    setFast(true); fastUntil = millis() + 5000;
    fileBusy = true; getOff = off; getLen = len;
    xTaskCreatePinnedToCore(getTask, "bleget", 4096, nullptr, 1, nullptr, 0);
}

} // namespace ble
