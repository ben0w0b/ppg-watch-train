/**
 * QMI8658C 6 軸 IMU —— 加速度計 + 陀螺儀
 *
 * Waveshare ESP32-S3-Touch-LCD-1.28 板載,Bus0 (SDA=6, SCL=7) 位址 0x6B。
 *
 * 走 LovyanGFX 的 I2C 驅動 (lgfx::i2c),不用 Arduino Wire:
 * 觸控 CST816S 在 LovyanGFX 設定裡是 i2c_port=0,LovyanGFX 用自己的驅動佔住 I2C0;
 * Arduino Wire 也用 I2C0,兩個驅動搶同一個周邊 —— 實測 Wire 全部逾時 (Error 263),
 * 而且逾時會把取樣任務卡住。ESP32-S3 只有兩個 I2C,I2C1 已給 PPG hub,
 * 所以 IMU 只能和觸控共用 LovyanGFX 的驅動。因此 begin() 必須在 lcd.init() 之後呼叫。
 *
 * 輸出單位換算成 WISDM 資料集的單位:加速度 m/s² (含重力)、角速度 rad/s。
 */
#pragma once

#include <Arduino.h>
#define LGFX_USE_V1
#include <LovyanGFX.hpp>

static constexpr int     QMI_PORT = 0;        // 與觸控相同的 i2c_port
static constexpr int     QMI_ADDR = 0x6B;
static constexpr uint32_t QMI_FREQ = 400000;

static constexpr uint8_t QMI_WHO_AM_I = 0x00;   // 應回傳 0x05
static constexpr uint8_t QMI_CTRL1    = 0x02;   // bit6 = 位址自動遞增
static constexpr uint8_t QMI_CTRL2    = 0x03;   // 加速度 aFS[6:4] aODR[3:0]
static constexpr uint8_t QMI_CTRL3    = 0x04;   // 陀螺儀 gFS[6:4] gODR[3:0]
static constexpr uint8_t QMI_CTRL7    = 0x08;   // bit0 aEN, bit1 gEN
static constexpr uint8_t QMI_AX_L     = 0x35;   // AX..GZ 連續 12 bytes

static constexpr uint8_t QMI_FIFO_WTM   = 0x13;
static constexpr uint8_t QMI_FIFO_CTRL  = 0x14;   // bit[1:0] 模式 0 bypass/1 fifo/2 stream, bit[3:2] 深度 16/32/64/128, bit7 FIFO_RD_MODE
static constexpr uint8_t QMI_FIFO_CNT   = 0x15;   // 樣本數 (16-bit 字數) 低 8 位
static constexpr uint8_t QMI_FIFO_STAT  = 0x16;   // bit[1:0] 樣本數高位, bit5 溢位
static constexpr uint8_t QMI_FIFO_DATA  = 0x17;
static constexpr uint8_t QMI_CTRL9      = 0x0A;
static constexpr uint8_t QMI_STATUSINT  = 0x2D;   // bit7 CmdDone
static constexpr uint8_t QMI_CMD_RST_FIFO = 0x04, QMI_CMD_REQ_FIFO = 0x05, QMI_CMD_ACK = 0x00;
// 2026-09-18 ODR 由 224 Hz 降到 56 Hz:改用 FIFO 每 200 ms 批次讀 (約 11 筆) 再抽成 20 Hz,主迴圈不必每 50 ms 醒來
static constexpr uint8_t QMI_ODR = 0x07;                       // 56.05 Hz
static constexpr float   QMI_ODR_HZ = 56.05f;
static constexpr uint8_t QMI_ACC_CFG = (0x01 << 4) | QMI_ODR;   // ±4g
static constexpr uint8_t QMI_GYR_CFG = (0x05 << 4) | QMI_ODR;   // ±512 dps
static constexpr float QMI_ACC_LSB_PER_G   = 8192.0f;
static constexpr float QMI_GYR_LSB_PER_DPS = 64.0f;
static constexpr float G_TO_MS2 = 9.80665f;
static constexpr float DPS_TO_RADS = 0.017453292f;

struct Qmi8658 {
    bool ok = false;
    uint8_t ctrl7 = 0x01;                          // 0x01 只開加速度計、0x03 加陀螺儀 (GYRO,0|1)

    static uint8_t r8(uint8_t reg)
    {
        auto r = lgfx::i2c::readRegister8(QMI_PORT, QMI_ADDR, reg, QMI_FREQ);
        return r.has_value() ? r.value() : 0xFF;
    }
    static bool w8(uint8_t reg, uint8_t v)
    {
        return lgfx::i2c::writeRegister8(QMI_PORT, QMI_ADDR, reg, v, 0, QMI_FREQ).has_value();
    }

    /* 必須在 lcd.init() 之後呼叫 (I2C0 由 LovyanGFX 初始化)。 */
    bool begin()
    {
        uint8_t id = r8(QMI_WHO_AM_I);
        if (id != 0x05) {
            Serial.printf("# QMI8658 WHO_AM_I=0x%02X (應為 0x05) — IMU 不可用\n", id);
            return ok = false;
        }
        w8(QMI_CTRL1, 0x40);
        w8(QMI_CTRL2, QMI_ACC_CFG);
        w8(QMI_CTRL3, QMI_GYR_CFG);
        w8(QMI_CTRL7, ctrl7);           // 2026-09-18 預設只開加速度計 (陀螺儀約 1 mA;HAR 純加速度 LOSO 91.0% 不比 acc+gyr 差)
        delay(20);
        Serial.printf("# QMI8658 就緒 (lgfx i2c port %d) CTRL2=0x%02X CTRL3=0x%02X CTRL7=0x%02X\n",
                      QMI_PORT, r8(QMI_CTRL2), r8(QMI_CTRL3), r8(QMI_CTRL7));
        return ok = true;
    }

    /* acc[3] m/s²,gyr[3] rad/s。失敗回傳 false 並保持輸出不變。 */
    bool read(float *acc, float *gyr)
    {
        if (!ok) return false;
        uint8_t b[12];
        if (!lgfx::i2c::readRegister(QMI_PORT, QMI_ADDR, QMI_AX_L, b, 12, QMI_FREQ).has_value())
            return false;
        for (int i = 0; i < 3; i++) {
            int16_t a = (int16_t)((b[2*i+1] << 8) | b[2*i]);
            int16_t g = (int16_t)((b[6+2*i+1] << 8) | b[6+2*i]);
            rawA[i] = a; rawG[i] = g;                 // 原始計數,二進位快取直接存這個 (無損)
            acc[i] = a / QMI_ACC_LSB_PER_G * G_TO_MS2;
            gyr[i] = g / QMI_GYR_LSB_PER_DPS * DPS_TO_RADS;
        }
        return true;
    }
    int16_t rawA[3] = {0, 0, 0}, rawG[3] = {0, 0, 0};   // 最近一次 read() / 解析的原始值

    /* ---- FIFO 批次讀 (QST 參考流程:CTRL9 REQ_FIFO → 等 CmdDone → ACK → 讀 FIFO_DATA → 清 RD_MODE) ---- */
    bool cmd9Req(uint8_t cmd)                     // 送 CTRL9 命令並等 CmdDone
    {
        if (!w8(QMI_CTRL9, cmd)) return false;
        uint32_t t0 = millis();
        while (!(r8(QMI_STATUSINT) & 0x80)) { if (millis() - t0 > 20) return false; delayMicroseconds(200); }
        return true;
    }
    bool cmd9Ack()                                // ACK 並等 CmdDone 清掉
    {
        w8(QMI_CTRL9, QMI_CMD_ACK);
        uint32_t t0 = millis();
        while (r8(QMI_STATUSINT) & 0x80) { if (millis() - t0 > 20) return false; delayMicroseconds(200); }
        return true;
    }
    bool cmd9(uint8_t cmd) { return cmd9Req(cmd) && cmd9Ack(); }
    bool fifoBegin()
    {
        if (!ok) return false;
        w8(QMI_CTRL7, 0x00); delay(5);              // 先關感測器再設 FIFO,最後再開 (參考驅動的順序)
        w8(QMI_FIFO_WTM, 32);
        w8(QMI_FIFO_CTRL, (0x02 << 2) | 0x02);      // 64 筆深、stream 模式 (滿了丟最舊,不會卡住)
        bool r = cmd9(QMI_CMD_RST_FIFO);
        w8(QMI_CTRL7, ctrl7); delay(5);
        Serial.printf("# QMI8658 FIFO %s (CTRL=0x%02X)\n", r ? "on" : "reset cmd failed", r8(QMI_FIFO_CTRL));
        return r;
    }
    void fifoEnd() { w8(QMI_FIFO_CTRL, 0x00); }
    void setGyro(bool on) { ctrl7 = on ? 0x03 : 0x01; w8(QMI_CTRL7, ctrl7); }
    void fifoDebug()                              // IMUDBG:印 FIFO 暫存器與前 24 bytes (除錯用)
    {
        uint8_t st = r8(QMI_FIFO_STAT), lo = r8(QMI_FIFO_CNT), ctrl = r8(QMI_FIFO_CTRL), c7 = r8(QMI_CTRL7), sti = r8(QMI_STATUSINT);
        Serial.printf("# IMUDBG stat=0x%02X cnt=%u ctrl=0x%02X ctrl7=0x%02X statusint=0x%02X\n", st, lo, ctrl, c7, sti);
        bool ok9 = cmd9Req(QMI_CMD_REQ_FIFO);
        uint8_t b[24] = {0};
        bool okr = lgfx::i2c::readRegister(QMI_PORT, QMI_ADDR, QMI_FIFO_DATA, b, 24, QMI_FREQ).has_value();
        cmd9Ack();
        w8(QMI_FIFO_CTRL, r8(QMI_FIFO_CTRL) & 0x7F);
        Serial.printf("# IMUDBG req=%d read=%d bytes:", ok9, okr);
        for (int i = 0; i < 24; i++) Serial.printf(" %02X", b[i]);
        Serial.println();
        float a[3], g[3]; read(a, g);
        Serial.printf("# IMUDBG poll acc=%.2f,%.2f,%.2f\n", a[0], a[1], a[2]);
    }
    /* 讀出 FIFO 內全部樣本,每筆呼叫 fn(acc m/s², gyr rad/s, rawA, rawG)。回傳筆數,失敗 -1。 */
    template <typename F> int fifoRead(F fn)
    {
        if (!ok) return -1;
        uint8_t st = r8(QMI_FIFO_STAT), lo = r8(QMI_FIFO_CNT);
        if (st == 0xFF || lo == 0xFF) return -1;
        int words = ((st & 0x03) << 8) | lo;          // 16-bit 字數
        const int SB = (ctrl7 & 0x02) ? 12 : 6;        // 每筆 acc3 = 6 bytes,開陀螺儀時 acc3+gyr3 = 12
        int n = words * 2 / SB;
        /* 實測 (2026-09-18) 讀到 FIFO 尾端時最後一個 byte 會變成填充字 (0x80),整批最後一筆的 z 軸就壞掉;
         * 所以永遠留最後一筆在 FIFO 裡不讀,下一批再拿 (延遲 200 ms,不掉樣本)。 */
        n -= 1;
        if (n <= 0) return 0;
        if (n > 64) n = 64;
        static uint8_t buf[64 * 12];
        if (!cmd9Req(QMI_CMD_REQ_FIFO)) return -1;     // 2026-09-18:讀資料要在 ACK 之前,ACK 後再讀會拿到 0x8000 填充字
        bool okr = lgfx::i2c::readRegister(QMI_PORT, QMI_ADDR, QMI_FIFO_DATA, buf, n * SB, QMI_FREQ).has_value();
        cmd9Ack();
        w8(QMI_FIFO_CTRL, r8(QMI_FIFO_CTRL) & 0x7F);  // 離開 FIFO_RD_MODE
        if (!okr) return -1;
        int good = 0;
        for (int k = 0; k < n; k++) {
            const uint8_t *b = buf + k * SB;
            bool filler = true;                        // 0x8000 / 0x7FFF = FIFO 空的填充字,不是樣本
            for (int i = 0; i < SB; i += 2) { uint16_t w = (uint16_t)((b[i+1] << 8) | b[i]); if (w != 0x8000 && w != 0x7FFF) { filler = false; break; } }
            if (filler) continue;
            good++;
            float acc[3], gyr[3] = {0, 0, 0};
            for (int i = 0; i < 3; i++) {
                int16_t a = (int16_t)((b[2*i+1] << 8) | b[2*i]);
                int16_t g = SB == 12 ? (int16_t)((b[6+2*i+1] << 8) | b[6+2*i]) : 0;
                rawA[i] = a; rawG[i] = g;
                acc[i] = a / QMI_ACC_LSB_PER_G * G_TO_MS2;
                gyr[i] = g / QMI_GYR_LSB_PER_DPS * DPS_TO_RADS;
            }
            fn(acc, gyr, rawA, rawG);
        }
        return good;
    }
};
