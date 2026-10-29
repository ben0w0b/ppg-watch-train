/**
 * PPG Watch — Waveshare ESP32-S3-Touch-LCD-1.28 + SparkFun MAX30101/MAX32664
 *
 * 綠光 PPG 版本 (手腕量測最佳化):
 *  - MAX30101 multi-LED 模式,只啟用 SLOT1 = 綠光;紅光與 IR 完全關閉
 *  - fs = 100 Hz (2026-09-18 批次 FIFO 讀取實測 hub 吐 99.4 筆/s、重複值 11–15%;HUBRATE,50 時吐 50.0 筆/s,I2C 讀取 7.9→2.0 ms/批;停頓由內插補齊)
 *  - 0.7–3.5 Hz 帶通濾波 (手腕用窄帶,更能壓制動作假影)
 *  - 心率主要由「階層式視窗」估測 (30s 外層 / 4s 子視窗,品質加權中位數)
 *    實測真實資料:階層式 83-86 穩定,自相關則在 48-83 間擺盪
 *  - 手腕灌流量僅指尖的 1/10,時域尋峰會失效
 *    (模擬:AC/DC=0.2% 時尋峰誤差 47bpm,自相關 2bpm)
 *  - LCD 顯示波形 + BPM + 訊號品質;觸控切換頁面
 *  - 取樣任務固定於核心 0 (100Hz 精準時序),繪圖/觸控在核心 1
 *  - 尋峰改為三路投票 (振幅 / 微分積分 / 二階微分,2 票確認)
 *  - Serial CSV: green,filt,bpm,contact,quality,hierbpm,peakbpm
 *
 * 接線 (SH1.0 外接排線):
 *   SDA=GPIO15  SCL=GPIO16  RESET=GPIO17  MFIO=GPIO18
 */

#include <Wire.h>
#include <SparkFun_Bio_Sensor_Hub_Library.h>

#include "find_peaks.h"      // scipy find_peaks 的 C++ 移植 (與 PC 端共用同一份)
#include "ppg_chain.h"       // 重取樣 + Butterworth + MA3 (與 PC 端同一條鏈)
#include "hrv_features.h"    // 11 維時域 + 7 維頻域 HRV 特徵
#include "stress_infer.h"    // LDA 推論 (三個韌體共用)
#include "stress_calib.h"    // 個人化判定線 (由 tools/calibrate.py 產生)
#include "cnn_infer.h"       // tiny 1D-CNN 不逐拍推論 (tools/export_tiny_cnn.py 產生 cnn_model.h)
#include "stft_mask.h"       // 走動時的 ACC 頻譜遮罩 (M9a),與 tools/stft_mask_ref.py 一對一
#include "qmi8658.h"         // 板載 IMU (加速度 + 陀螺儀)
#include "har_features.h"    // 活動辨識特徵 (與 tools/train_har.py 對應)
#include "har_infer.h"       // 活動辨識 LDA 推論 (診斷用分數)
#include "har_forest_infer.h" // 活動辨識隨機森林推論 (實際判定,LOSO 90.2%)
#include <LittleFS.h>          // 板上錄音 (拔線走路用)
#include <esp_system.h>
#include <driver/gpio.h>
#include <esp_heap_caps.h>
#include <time.h>
#include <sdkconfig.h>
#if CONFIG_PM_ENABLE
#include <esp_pm.h>            // 路線 A (pioarduino hybrid 環境):自動 light sleep + 動態調頻
#include <esp_sleep.h>
#endif
#ifndef BLE_ENABLED
#define BLE_ENABLED 1
#endif
#if BLE_ENABLED
#include "ble_link.h"          // 手機 app 經 BLE 操控 (指令 / 狀態 / 波形 / 檔案)
#endif
static void logPush(const char *s, size_t n);   // 定義在後面的板上錄音區塊
static void logPpgBin(uint32_t ms, uint32_t green, uint32_t ir);              // 二進位快取:PPG 區塊編碼 (核心 0 專用)
static void logImuBin(uint32_t ms, const int16_t *a, const int16_t *g);       // 二進位快取:IMU 紀錄 (0xF2 只有加速度;GYRO,1 時 0xF1 六軸)
#include "qmi8658.h"
static Qmi8658 imu;
static uint8_t logFmt = 1;
static volatile bool serialQuiet = false;         // 序列埠 DUMP 進行中:暫停週期性診斷輸出,免得絞進二進位串流                        // 快取格式:0 純文字 (舊)、1 二進位 (預設)、2 兩者都寫 (驗證用)
static bool logRec = false;                      // 快取常駐寫入中 (開機後永遠 true)
static bool logMarked = false; static uint32_t logMarkMs = 0;   // 使用者按了 REC (段落標記) 到 STOP 之間:畫面與 app 顯示 REC 的依據
static void fastSet(bool on);   // 校正模式:每秒推論 (定義在壓力區塊之後)
static void hubSetBatch(uint32_t ms); // hub 批次讀取週期 (定義在取樣任務之前)
static void batLogDump();             // BATLOG:印出 /bat.csv
static void batFileAppend(uint32_t now); // /bat.csv 追加一筆 (電池 + 螢幕/BLE/傳檔/配戴/LED 累計值)
static volatile int irMode = 1;             // IR,0|1|2:參考通道 (實際是綠光 LED) 0 關 (配戴只剩動作判斷)、1 常亮 (預設;綠光太弱,間歇 10 s 視窗 corr 只有 0.2–0.67 不可靠,2026-09-18 使用者決定)、2 間歇 (相位 17.5–30 s 亮)
static volatile int irWant = 1;             // 目前應該亮不亮 (由取樣任務依 irMode 與相位算)
static constexpr uint32_t IR_CYCLE_MS = 30000, IR_ON_FROM_MS = 17500;   // 參考燈相位 17.5–30 s 亮 (42%);配戴相關在相位 29.5 s 用最後 10 s 算 (5 s 視窗實測 corr 0.2–0.95 不穩,10 s 才夠)
static constexpr int      WEAR_DUTY_N = 1000;  // 相位 19–29 s 的 10 s (亮燈後留 1.5 s 給濾波器暫態)
static volatile uint16_t hubPwReq = 0;      // PW,<us>:要求改 LED 脈寬 (69/118/215/411 µs = 15–18 bit),同樣由取樣任務套用
static volatile uint16_t hubRateReq = 0;    // HUBRATE,<hz>:要求改 hub 取樣率,由取樣任務套用 (同一條 I2C 只能在那裡動)
static uint32_t hubGot = 0;                 // 這 5 秒 hub 實際吐出的筆數 (量真實取樣率用)
static void pmBusEnter(); static void pmBusExit(); static void pmForce(bool on); static void pmNoSleepSet(bool on); static void pmWakeLock();
static constexpr int PIN_UART_RX = 44;
static bool usbPresent = true;
static int  usbForce = -1;                        // USB,0|1 強制 (耗電測試:電池充飽 >4.05 V 時拔線不會被判成電池,整段不睡);USB,A 回自動;不存 NVS                    // 近 2 分鐘有序列埠資料,或電壓跳變判定為充電 (RX 線電位不可靠:CH343 由 3.3 V 供電,沒 USB 也是高)
static uint32_t lastSerialRxMs = 0;
static bool liveSerial = true, liveForce = false;   // 即時 100 Hz 序列輸出:沒 USB 就關 (VERB,1 強制開)   // 電源管理鎖 (定義在省電區塊;沒開 PM 時是空函式)
static void screenWake(); static void screenSetSleep(uint32_t sec); static const char *batStat(); static void timeSet(uint32_t epoch, int tzMin); static void lpSet(bool on); static void batCal(int measuredMv); static void rotSet(int r);   // 定義在電池/螢幕休眠區塊
static uint32_t logTotalBytes(); static size_t logReadAt(uint32_t, uint8_t *, size_t); static void logFlush(bool);
// 配戴偵測的狀態 (wearUpdate 在後面,harUpdate 先用到)
static constexpr uint32_t WEAR_TIMEOUT_MS = 180000;
static constexpr float    WEAR_MOTION_STD = 0.08f;   // |a| 2 秒標準差 (m/s²):桌上中位 0.011、最大 0.059;真正靜坐時 2 秒視窗也可能只有 0.02–0.05
// 脈波判據改用壓力鏈的 RR 數 (階層式品質桌上雜訊 0.33–0.40、戴著也 0.4 上下,分不開):
// 120 s 視窗戴著時通常 100 拍以上;桌上雜訊的數值待實測 (WEAR_MIN_RR 先放 60)。
static constexpr int      WEAR_MIN_RR     = 60;      // (保留但不再使用:桌上雜訊 120 s 也有 160–188 個 RR)
static int      wearLastNRR = 0; static uint32_t wearLastNRRms = 0;
// 脈波判據 (2026-09-10 實測):綠光與 IR 帶通後 30 秒的相關係數 —— 貼著皮膚 0.82–0.98,放桌上 -0.10–0.01
// 2026-09-15 各 10 分鐘實測:戴著 corr 最低 0.75 (p10 0.88、中位 0.94);放桌上最高 0.26 (中位 0.01) → 門檻 0.50 兩邊都留有餘裕
static constexpr float    WEAR_CORR_MIN   = 0.70f;   // 2026-09-15 改用截幅相關後提高到 0.7 (桌上拿取暫態截幅後最高 0.64)
// 沒脈波多久就算沒戴:corr 用 30 s 緩衝,拿下後約 15 s 掉到門檻以下,再 30 s → 約 45 s 判 OFF (原本要等 3 分鐘沒動作)
static constexpr uint32_t WEAR_NOPULSE_MS = 30000;
static constexpr int      WEAR_CORR_N     = 30 * 100;   // 30 s @ 100 Hz
static float   *wearG = nullptr, *wearI = nullptr;      // 帶通後的綠與 IR 環形緩衝 (PSRAM)
static int      wearHead = 0, wearFill = 0;
static float    wearCorr = 0;
static constexpr uint32_t WEAR_PAUSE_MS = 180000;   // 未配戴狀態持續超過 3 分鐘才停止壓力判定 (使用者規格)
static uint32_t wornOffMs = 0;                       // 進入未配戴狀態的時刻
static uint32_t lastMotionMs = 0, lastPulseMs = 0;
static bool     wearMotionEver = false;   // 開機後是否真的偵測過動作 (開機時 lastMotionMs 只是初始值)
static bool     wornForce = false;        // WORN,1:強制視為配戴 (耗電測試用:外殼裝電池後戴不上,放桌上也要讓 LED 與推論照常跑);不存 NVS,重開機即恢復
static bool     worn = true;
static float    wearAccStd = 0;


#include <algorithm>
#define LGFX_USE_V1
#include <LovyanGFX.hpp>

// 設為 1 可在開機時執行自我診斷:掃 LED 電流,確認讀值真的會跟著變。
// 若讀值紋風不動代表資料路徑壞掉 (通常是動到 hub 代管的暫存器所致)。
#define SELF_TEST 0

// 壓力辨識開關。設 0 只留心率量測 (env:bpm_only 會定義成 0)。
// 用編譯旗標而不是另開一份原始碼 —— 兩份會各自演化然後悄悄分岔。
#ifndef STRESS_ENABLED
#define STRESS_ENABLED 1
#endif
// 活動辨識 (坐/站/走/跑),獨立於壓力辨識。
#ifndef HAR_ENABLED
#define HAR_ENABLED 1
#endif

// ---------- 參數 ----------
static constexpr float FS = 100.0f;   // 送入處理鏈的取樣率 (Hz),停頓處由內插補齊
// 每筆樣本的間隔。取樣任務一律用微秒計時 (FreeRTOS 的 1ms tick 解析度不足以
// 表達非整數毫秒的週期,例如 64Hz 的 15.625ms)。
static constexpr uint32_t SAMPLE_US = (uint32_t)(1000000.0f / FS + 0.5f);
static constexpr uint32_t SAMPLE_MS = (uint32_t)(1000.0f / FS + 0.5f);
static constexpr float HPF_FC = 0.7f; // 帶通下限 (Hz) — 手腕用 0.7Hz 更能壓基線飄移
static constexpr float LPF_FC = 3.5f; // 帶通上限 (Hz) — 3.5Hz = 210bpm

static constexpr int PIN_PPG_SDA = 15;
static constexpr int PIN_PPG_SCL = 16;
static constexpr int PIN_PPG_RST = 17;
static constexpr int PIN_PPG_MFIO = 18;

// MAX30101 暫存器 (透過 MAX32664 hub 轉寫)
static constexpr uint8_t REG_MODE_CONFIG = 0x09; // 0x07 = multi-LED mode
static constexpr uint8_t REG_SLOT_1_2 = 0x11;    // SLOT2[6:4] | SLOT1[2:0]
static constexpr uint8_t REG_SLOT_3_4 = 0x12;
static constexpr uint8_t REG_LED1_PA = 0x0C;     // 紅光振幅
static constexpr uint8_t REG_LED2_PA = 0x0D;     // IR 振幅
static constexpr uint8_t REG_LED3_PA = 0x0E;     // 綠光振幅暫存器「位址」(固定,勿改)
// ⚠ 以下兩個暫存器「不可寫入」。MAX30101 是由 MAX32664 hub 代管的,改寫這兩個
// 會讓 hub 的 FIFO 解析對不上,readSensor() 從此只回傳同一筆舊值 (資料看似仍在
// 流動,數值卻是死的)。僅保留位址供 SELF_TEST 的診斷程式使用。
static constexpr uint8_t REG_FIFO_CONFIG = 0x08; // SMP_AVE[7:5] | ROLLOVER[4] | A_FULL[3:0]
static constexpr uint8_t REG_SPO2_CONFIG = 0x0A; // ADC_RGE[6:5] | SR[4:2] | LED_PW[1:0]
static constexpr uint8_t FIFO_CFG_AVG4 = 0x50;   // (診斷用,勿套用)
static constexpr uint8_t SPO2_CFG_WRIST = 0x2F;  // (診斷用,勿套用)
// slot 編碼: 001=LED1(紅), 010=LED2(IR), 011=LED3(綠), 000=停用
// 只啟用 SLOT1=綠光,紅光與 IR 完全關閉 (少一個 slot 也讓 FIFO 讀取更快)
static constexpr uint8_t SLOT_GREEN_ONLY = 0x03;
static constexpr uint8_t SLOT_GREEN_IR   = 0x23;   // slot1=綠(3) slot2=IR(2):hub raw 模式只送兩個時槽 (2026-09-10 實測)
static constexpr uint8_t IR_BRIGHTNESS   = 0x7F;   // IR 只當運動參考,不必開滿
static volatile uint32_t lastIr = 0;                // 最近一筆 IR 原始值 (與綠光同一取樣)

// ★ 要調綠光亮度改這裡:0x00–0xFF ≈ 0–51mA (0xBF≈38mA, 0x7F≈25mA)
static constexpr uint8_t GREEN_BRIGHTNESS = 0xFF; // 最大 ~51mA (訊號太弱時先開滿)

// ---------- LovyanGFX 板級設定 (Waveshare ESP32-S3-Touch-LCD-1.28) ----------
class LGFX_Watch : public lgfx::LGFX_Device
{
    lgfx::Panel_GC9A01 _panel;
    lgfx::Bus_SPI _bus;
    lgfx::Light_PWM _light;
    lgfx::Touch_CST816S _touch;

public:
    LGFX_Watch()
    {
        {
            auto cfg = _bus.config();
            cfg.spi_host = SPI2_HOST;
            cfg.spi_mode = 0;
            cfg.freq_write = 80000000;
            cfg.freq_read = 20000000;
            cfg.dma_channel = SPI_DMA_CH_AUTO;
            cfg.pin_sclk = 10;
            cfg.pin_mosi = 11;
            cfg.pin_miso = 12;
            cfg.pin_dc = 8;
            _bus.config(cfg);
            _panel.setBus(&_bus);
        }
        {
            auto cfg = _panel.config();
            cfg.pin_cs = 9;
            cfg.pin_rst = 14;
            cfg.panel_width = 240;
            cfg.panel_height = 240;
            cfg.readable = false;
            cfg.invert = true;
            _panel.config(cfg);
        }
        {
            auto cfg = _light.config();
            cfg.pin_bl = 2;
            cfg.freq = 12000;
            cfg.pwm_channel = 7;
            _light.config(cfg);
            _panel.setLight(&_light);
        }
        {
            auto cfg = _touch.config();
            cfg.pin_sda = 6;
            cfg.pin_scl = 7;
            cfg.pin_int = 5;
            cfg.pin_rst = 13;
            cfg.i2c_port = 0;
            cfg.i2c_addr = 0x15;
            cfg.freq = 400000;
            cfg.x_min = 0;
            cfg.x_max = 239;
            cfg.y_min = 0;
            cfg.y_max = 239;
            _touch.config(cfg);
            _panel.setTouch(&_touch);
        }
        setPanel(&_panel);
    }
};

static LGFX_Watch lcd;
static LGFX_Sprite frame(&lcd);

// ---------- 濾波器:RBJ biquad (transposed direct form II) ----------
struct Biquad
{
    float b0, b1, b2, a1, a2;
    float z1 = 0, z2 = 0;

    void lowpass(float fs, float fc, float Q = 0.7071f)
    {
        float w = 2.0f * PI * fc / fs, c = cosf(w), s = sinf(w), al = s / (2 * Q);
        float a0 = 1 + al;
        b0 = (1 - c) / 2 / a0; b1 = (1 - c) / a0; b2 = b0;
        a1 = -2 * c / a0; a2 = (1 - al) / a0;
    }
    void highpass(float fs, float fc, float Q = 0.7071f)
    {
        float w = 2.0f * PI * fc / fs, c = cosf(w), s = sinf(w), al = s / (2 * Q);
        float a0 = 1 + al;
        b0 = (1 + c) / 2 / a0; b1 = -(1 + c) / a0; b2 = b0;
        a1 = -2 * c / a0; a2 = (1 - al) / a0;
    }
    float process(float x)
    {
        float y = b0 * x + z1;
        z1 = b1 * x - a1 * y + z2;
        z2 = b2 * x - a2 * y;
        return y;
    }
    void reset() { z1 = z2 = 0; }
    // 預載狀態:定值輸入 x 時輸出立即為 0,消除 DC 階躍造成的長暫態
    void primeDC(float x) { z2 = b2 * x; z1 = b1 * x + z2; }
};

static Biquad hpf, lpf;

/* ---------- demo4 風格的 BPM 偵測 (對照用) -----------------------------
 * 參考 demo4.ino (XD-58C 類比脈搏感測器) 的作法,移植到 100Hz:
 *   單極高通 0.5Hz → 單極低通 5Hz → EMA 包絡 → 門檻 = 0.5 x 包絡 → 尋峰
 * 與本專案原本的作法最大的差異是「包絡用 EMA 而非取最大值」,受單次假影的
 * 影響較小;但 EMA 包絡在訊號忽強忽弱時追得比較慢。
 */
static constexpr float D4_FC_HP = 0.5f;
static constexpr float D4_FC_LP = 5.0f;
static constexpr float D4_ALPHA_ENV = 0.1f;
static constexpr float D4_THR_K = 0.5f;
static constexpr float D4_ALPHA_BPM = 0.2f;

static float d4_xPrev = 0, d4_hpPrev = 0, d4_lpPrev = 0, d4_env = 0;
static float d4_s1 = 0, d4_s2 = 0;
static uint32_t d4_lastPeak = 0;
static float d4_bpmEma = 0;
static uint16_t d4Bpm = 0;

static void d4Reset()
{
    d4_xPrev = d4_hpPrev = d4_lpPrev = d4_env = 0;
    d4_s1 = d4_s2 = 0;
    d4_lastPeak = 0;
    d4_bpmEma = 0;
    d4Bpm = 0;
}

static void d4Feed(float x, uint32_t nowMs)
{
    const float aHp = expf(-2.0f * PI * D4_FC_HP / FS);
    const float dt = 1.0f / FS;
    const float rcLp = 1.0f / (2.0f * PI * D4_FC_LP);
    const float aLp = dt / (rcLp + dt);

    float hp = aHp * (d4_hpPrev + x - d4_xPrev);
    d4_hpPrev = hp; d4_xPrev = x;
    float lp = d4_lpPrev + aLp * (hp - d4_lpPrev);
    d4_lpPrev = lp;

    d4_env = (1 - D4_ALPHA_ENV) * d4_env + D4_ALPHA_ENV * fabsf(lp);
    float thr = D4_THR_K * d4_env;

    if (d4_s1 > d4_s2 && d4_s1 > lp && d4_s1 > thr) {
        uint32_t ibi = nowMs - d4_lastPeak;
        if (d4_lastPeak > 0 && ibi >= 300 && ibi <= 1500) {
            float inst = 60000.0f / (float)ibi;
            d4_bpmEma = (d4_bpmEma == 0) ? inst
                                         : (1 - D4_ALPHA_BPM) * d4_bpmEma + D4_ALPHA_BPM * inst;
            d4Bpm = (uint16_t)(d4_bpmEma + 0.5f);
        }
        d4_lastPeak = nowMs;
    }
    d4_s2 = d4_s1; d4_s1 = lp;
}

// ---------- 診斷計數器 ----------
static volatile uint32_t acUpdates = 0;    // 自相關實際完成更新的次數

// ---------- PPG 感測器 ----------
static SparkFun_Bio_Sensor_Hub bioHub(PIN_PPG_RST, PIN_PPG_MFIO);
static bool sensorOk = false;

// 波形環形緩衝 (濾波後訊號)
static constexpr int WAVE_LEN = 220;
static float waveBuf[WAVE_LEN] = {0};
static int waveHead = 0;

// ---------- 峰值檢測參數 (經合成訊號模擬調校) ----------
static constexpr float THR_K = 0.35f;       // 門檻 = THR_K x 近期峰值中位數
static constexpr float BOOT_DECAY = 0.92f;  // 啟動期包絡衰減 (越小越快回到靈敏)
static constexpr uint32_t BLANK_MS = 400;   // 手指剛放上的空白期

// 心率估測狀態
static uint32_t lastBeatMs = 0;
static uint16_t ibiBuf[5] = {0}; // 最近 5 次心跳間隔 (ms)
static int ibiCount = 0;
static uint16_t bpm = 0;
static float dcBase = 0;         // 環境光地板基準 (不對稱追蹤)
static constexpr int WARMUP_N = (int)(2 * FS); // 開機後 2 秒不判定接觸
static int warmup = 0;
static uint32_t blankUntil = 0;

// 2026-09-22 使用者要求:螢幕只留 時間、電量、連線圖示、BPM (時鐘頁);心率+壓力頁與波形頁已移除

// ---------- 顏色 ----------
static constexpr uint16_t COL_BG = TFT_BLACK;
static constexpr uint16_t COL_WAVE = 0x07E0;
static constexpr uint16_t COL_HR = 0xFB2C;
static constexpr uint16_t COL_DIM = 0x8410;

static bool initSensor()
{
    Wire1.setBufferSize(1024);                              // 批次讀 hub FIFO 一次最多 84 筆 (1009 bytes),須在 begin 前設
    Wire1.begin(PIN_PPG_SDA, PIN_PPG_SCL, 400000);
    if (bioHub.begin(Wire1, PIN_PPG_RST, PIN_PPG_MFIO) != 0)
        return false;

    // 只要原始感測數據,不啟用 hub 演算法 (綠光模式下演算法不適用)
    if (bioHub.configSensor() != 0)
        return false;
    bioHub.setPulseWidth(215);   // 2026-09-18 由 411 改 215:LED 能量減半;手腕實測訊噪比/配戴相關/HR 無差 (PW,<us> 可即時改)
    // 只能設 100。實測設 400 雖然資料停頓變少,但會讓 hub 進入錯誤狀態:
    // 之後所有暫存器讀回都是 0xFE、寫入全部失效,結果退回 hub 預設的紅光+IR
    // 模式 (LED 掃描實測 IR delta=30032、綠光只有雜訊等級)。
    // hub 在 100 下實際只輸出約 50Hz 且會週期性停頓,改由韌體端內插處理。
    bioHub.setSampleRate(100);
    delay(1000);

    // 關閉 AGC,避免 hub 自動改寫 LED 電流設定
    bioHub.agcAlgoControl(0);
    delay(100);

    // multi-LED 模式:只開 SLOT1=綠光,資料進 bioData.irLed 欄位
    bioHub.writeRegisterMAX30101(REG_MODE_CONFIG, 0x07);
    bioHub.writeRegisterMAX30101(REG_SLOT_1_2, SLOT_GREEN_IR);   // 綠光 + IR 兩時槽 (IR 供多波長運動參考研究)
    bioHub.writeRegisterMAX30101(REG_SLOT_3_4, 0x00);
    bioHub.writeRegisterMAX30101(REG_LED3_PA, GREEN_BRIGHTNESS);
    bioHub.writeRegisterMAX30101(REG_LED1_PA, 0x00); // 紅光完全關閉
    bioHub.writeRegisterMAX30101(REG_LED2_PA, IR_BRIGHTNESS);

    // 注意:不可自行改寫 FIFO_CONFIG (硬體平均) 或 SPO2_CONFIG (內部取樣率)。
    // MAX30101 是由 MAX32664 hub 代管的,改動這兩個暫存器會讓 hub 的 FIFO 解析
    // 對不上,readSensor() 就一直回傳同一筆舊值 (實測:LED 電流從 0 掃到滿,
    // 讀值紋風不動)。降噪只能靠韌體端的數位濾波。
    delay(100);

    // 讀回驗證,方便從 serial 確認設定有生效
    Serial.printf("# MODE=0x%02X SLOT=0x%02X GREEN_PA=0x%02X\n",
                  bioHub.readRegisterMAX30101(REG_MODE_CONFIG),
                  bioHub.readRegisterMAX30101(REG_SLOT_1_2),
                  bioHub.readRegisterMAX30101(REG_LED3_PA));
    return true;
}

// ============ 自相關心率估測 (手腕主力演算法) ============
// 原理:PPG 是準週期訊號,把訊號與自身延遲 τ 相乘求和,τ = 一個心動週期時
// 相關值最大。雜訊不具週期性,在求和過程中被平均掉,因此在低訊噪比下
// 遠比「找單一個峰」穩健。
static constexpr int AC_WIN = (int)(10 * FS);       // 10 秒視窗
// 上限壓到 150bpm:靜態量測不會超過,放寬只是給雜訊機會被誤判成高心率
static constexpr int LAG_MIN = (int)(FS * 60 / 150); // 150 bpm
static constexpr int LAG_MAX = (int)(FS * 60 / 35);  // 35 bpm
// 取「最短延遲的顯著峰」時的門檻:相關值需達全域最大值的這個比例。
// 太低會誤取諧波 (心率偏高兩倍),太高會退回全域最大值 (卡在次諧波)。
static constexpr float SUBHARM_K = 0.85f;
// 已移除品質門檻:心率無條件更新。acQuality 仍會計算並輸出到 serial 供分析,
// 但不再用來擋讀值或改變顯示。

// 取樣在核心 0、繪圖在核心 1,共用緩衝需保護
static portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;

static float acBuf[AC_WIN] = {0};
static int acHead = 0;
static int acFill = 0;
static uint16_t acBpm = 0;    // 自相關估出的心率
static float acQuality = 0;   // 0–1,訊號品質 (最佳相關係數)

static void acPush(float v)
{
    acBuf[acHead] = v;
    acHead = (acHead + 1) % AC_WIN;
    if (acFill < AC_WIN) acFill++;
}

// 每 1 秒呼叫一次,約需 2ms (240MHz + FPU)
static void acUpdate()
{
    static float seg[AC_WIN];

    // 只在複製時上鎖,後面的重運算不佔用取樣核心
    portENTER_CRITICAL(&dataMux);
    bool ready = (acFill >= AC_WIN);
    if (ready) {
        int h = acHead;
        for (int i = 0; i < AC_WIN; i++)
            seg[i] = acBuf[(h + i) % AC_WIN]; // 還原成時間順序
    }
    portEXIT_CRITICAL(&dataMux);
    if (!ready) return;

    float mean = 0;
    for (int i = 0; i < AC_WIN; i++) mean += seg[i];
    mean /= AC_WIN;

    float e0 = 0;
    for (int i = 0; i < AC_WIN; i++) { seg[i] -= mean; e0 += seg[i] * seg[i]; }
    if (e0 <= 0) { acQuality = 0; return; }

    /* 掃描所有延遲並保留完整剖面。
     *
     * ⚠ 這裡不可以直接取「全域最大值」。週期訊號在 2 倍週期處的相關值幾乎
     * 和基頻一樣高 — 實測真實量測資料:
     *     延遲 62 → 96.8 bpm  相關值 0.291   (基頻,正確)
     *     延遲121 → 49.6 bpm  相關值 0.287   (2 倍週期)
     * 只差 1.4%,訊號稍有波動就會翻到次諧波,再被平滑鎖死 → 讀值卡在 48-50。
     *
     * 正確做法 (與音高偵測相同):取「最短延遲的顯著峰」。基頻一定出現在
     * 比它的所有倍數週期更短的延遲上。
     */
    static float rbuf[LAG_MAX - LAG_MIN + 1];
    const int nLag = LAG_MAX - LAG_MIN + 1;
    float rMax = 0;
    int lagAtMax = 0;
    for (int i = 0; i < nLag; i++) {
        int lag = LAG_MIN + i;
        float acc = 0;
        for (int j = 0; j < AC_WIN - lag; j++) acc += seg[j] * seg[j + lag];
        rbuf[i] = acc / e0;
        if (rbuf[i] > rMax) { rMax = rbuf[i]; lagAtMax = lag; }
    }

    acQuality = rMax;
    if (lagAtMax <= 0 || rMax <= 0) return;

    // 由短延遲往長延遲掃,取第一個達到 SUBHARM_K x 最大值的局部峰
    int bestLag = lagAtMax;
    int bestIdx = lagAtMax - LAG_MIN;
    for (int i = 1; i < nLag - 1; i++) {
        if (rbuf[i] >= SUBHARM_K * rMax && rbuf[i] >= rbuf[i - 1] && rbuf[i] >= rbuf[i + 1]) {
            bestLag = LAG_MIN + i;
            bestIdx = i;
            break;
        }
    }

    // 拋物線內插,細化到次取樣點精度 (提升心率解析度)
    float lag = (float)bestLag;
    if (bestIdx > 0 && bestIdx < nLag - 1) {
        float rB = rbuf[bestIdx - 1], rA = rbuf[bestIdx + 1], rC = rbuf[bestIdx];
        float denom = rB - 2 * rC + rA;
        if (fabsf(denom) > 1e-9f) {
            float delta = 0.5f * (rB - rA) / denom;
            if (fabsf(delta) < 1.0f) lag += delta;
        }
    }

    float hr = 60.0f * FS / lag;
    if (hr < 35 || hr > 220) return;
    acBpm = (acBpm == 0) ? (uint16_t)(hr + 0.5f)
                         : (uint16_t)(0.7f * acBpm + 0.3f * hr + 0.5f); // 平滑
    acUpdates++;
}

/* ============ 階層式視窗 BPM (主要顯示) ============================
 * 外層:30 秒視窗,每 1 秒更新一次
 * 內層:4 秒子視窗、步進 1 秒 → 27 個重疊子視窗,各自尋峰
 *
 * ⚠ 視窗長度直接換取反應速度與穩定度。合成訊號實測 (手腕一般訊噪比,
 *   10 個不同時間點的讀值標準差):
 *     30秒(28子窗) 誤差11.4 抖動0.6bpm | 10秒(8子窗) 誤差14.2 抖動1.1bpm
 *     5秒(3子窗)  誤差19.9 抖動4.6bpm
 *   目前設 30 秒:最穩定,但貼上後要等滿 30 秒才有第一個讀值。
 * 每個子視窗算出一個 BPM 與「峰品質」,最後取品質加權中位數。
 *
 * 峰品質 = 節律規律度 x 峰高一致性 x 突出度,三者皆為 0–1:
 *   節律規律度 = 1 - IBI 的變異係數    (心跳應該規律)
 *   峰高一致性 = 1 - 峰高的變異係數    (真實脈搏峰高相近)
 *   突出度     = 平均峰高 / 視窗 RMS   (峰要明顯高於背景雜訊)
 *
 * 模擬比較 (8 種情境平均心率誤差):階層式 19.8bpm、自相關 17.8bpm。
 * 兩者相當,階層式在基線飄移與晃動時較好、自相關在極弱訊號時較好。
 * 階層式的品質值鑑別力較佳:好訊號 0.61–0.95、壞訊號 0.28–0.46。
 */
static constexpr int HIER_OUTER = (int)(30 * FS); // 30 秒 (28 個子視窗)
/* 內層子視窗 4 秒。
 * 用實際錄製資料離線網格搜尋得到:3 秒時每秒估計值的標準差 12.6bpm,
 * 4 秒降到 4.2bpm(四分位距 23.6 → 4.3)。
 * 機制:心率約 70 時,3 秒只有 3.4 拍 = 2-3 個間隔,取中位數的樣本太少;
 * 4 秒有 4.6 拍 = 3-4 個間隔,穩定得多。心率越低差異越明顯。
 * 註:合成訊號的「平均」誤差反而偏好 3 秒 (7.2 vs 15.7),但那是被極端情境
 * (極弱訊號、劇烈晃動) 拉高的;「中位」誤差仍是 4 秒較好 (1.4 vs 2.1)。 */
static constexpr int HIER_INNER = (int)(4 * FS);  // 4 秒子視窗
static constexpr int HIER_STEP  = (int)(1 * FS);  // 步進 1 秒
static constexpr int HIER_SUBS  = (HIER_OUTER - HIER_INNER) / HIER_STEP + 1;
static constexpr int MAX_PK = 16; // 3 秒內最多可能的峰數 (150bpm 約 7.5 個)

static float hierBuf[HIER_OUTER];
static int hierHead = 0, hierFill = 0;
static uint16_t hierBpm = 0;
static float hierQuality = 0;

static void hierPush(float v)
{
    hierBuf[hierHead] = v;
    hierHead = (hierHead + 1) % HIER_OUTER;
    if (hierFill < HIER_OUTER) hierFill++;
}

static float meanOf(const float *a, int n)
{
    float s = 0; for (int i = 0; i < n; i++) s += a[i];
    return n ? s / n : 0;
}
static float stdOf(const float *a, int n)
{
    if (n < 2) return 0;
    float m = meanOf(a, n), s = 0;
    for (int i = 0; i < n; i++) s += (a[i] - m) * (a[i] - m);
    return sqrtf(s / n);
}
static float medianAny(float *a, int n) // 會就地排序
{
    for (int i = 0; i < n; i++)
        for (int j = i + 1; j < n; j++)
            if (a[j] < a[i]) { float t = a[i]; a[i] = a[j]; a[j] = t; }
    return n ? a[n / 2] : 0;
}

/* ---- SciPy find_peaks 的核心演算法 (移植自 scipy.signal) --------------
 * 三個步驟,與原本「大於前後鄰居且超過門檻」的簡易作法有三處實質差異:
 *
 * 1. 局部極大 (_local_maxima_1d):平台 (連續等值) 會取「中點」為峰位,
 *    原本的寫法會取平台的最後一點,峰位系統性偏移。
 * 2. 距離篩選 (_select_by_peak_distance):按峰高由高到低處理,保留最高的、
 *    刪掉其鄰近範圍內的其他峰。原本是「先到先得」,一個早出現的小峰會把
 *    後面真正的大峰擋掉。
 * 3. 突出度 (peak_prominences):峰相對於「兩側最低鞍點」的高度,而非絕對高度。
 *    這對 PPG 特別重要 —— 重搏波的絕對高度可能不低,但它坐落在收縮峰的
 *    斜坡上,突出度很小,因此能被正確排除。
 *
 * 合成訊號實測 (5 組種子 x 8 種情境 = 40 次):
 *   原本 0.35x最大值門檻     平均誤差 25.5 bpm,誤差>20 的有 23 次
 *   find_peaks 突出度 0.45   平均誤差 12.6 bpm,誤差>20 的有 13 次
 */
/* 突出度門檻以「標準差」為單位 (子視窗先做 z-score 標準化)。
 * 用標準差而非振幅範圍,是為了不受單一離群尖峰左右 —— 範圍由 max-min 決定,
 * 一個雜訊尖峰就能把門檻整個拉高。
 * 注意:標準化後若仍用「k x 振幅範圍」當門檻,兩者會互相抵銷、完全等價,
 * 所以門檻必須是絕對的 σ 值才有意義。
 * 合成訊號實測 (10 組種子 x 8 種情境 = 80 次):
 *   0.45 x 範圍  平均誤差 9.1bpm,誤差>20 有 21 次
 *   2.2σ        平均誤差 6.8bpm,誤差>20 有 11 次   (2.1-2.3σ 都優於前者) */
/* 單一 3 秒子視窗:z-score 標準化 → find_peaks 尋峰 → BPM 與品質 */
static void subWindow(const float *seg, int n, float *outBpm, float *outQ)
{
    *outBpm = 0; *outQ = 0;

    // --- z-score 標準化 ---
    float mu = 0;
    for (int i = 0; i < n; i++) mu += seg[i];
    mu /= n;
    float var = 0;
    for (int i = 0; i < n; i++) { float d = seg[i] - mu; var += d * d; }
    float sd = sqrtf(var / n);
    if (sd <= 1e-9f) return; // 平坦訊號,無從判斷

    static float z[HIER_INNER];
    for (int i = 0; i < n; i++) z[i] = (seg[i] - mu) / sd;

    int idx[MAX_PK];
    float amp[MAX_PK];
    // distance = 0 → 不做距離篩選。2.2σ 的突出度門檻已足以擋掉重搏波
    // (它坐落在收縮峰的斜坡上,突出度很小),實測取消後平均誤差 6.8→6.6,
    // 大誤差次數不變。
    int np = findPeaks(z, n, PROM_SIGMA, PEAK_DISTANCE, idx, amp, MAX_PK);
    if (np < 3) return;

    float ibis[MAX_PK]; int ni = 0;
    for (int i = 1; i < np; i++) {
        float ms = (idx[i] - idx[i - 1]) * 1000.0f / FS;
        if (ms >= 300 && ms <= 2000) ibis[ni++] = ms;
    }
    if (ni < 2) return;

    float ibiCopy[MAX_PK];
    memcpy(ibiCopy, ibis, ni * sizeof(float));
    float medIbi = medianAny(ibiCopy, ni);
    if (medIbi <= 0) return;
    float bpmv = 60000.0f / medIbi;
    if (bpmv < 35 || bpmv > 150) return;

    // --- 品質 (三項相乘) ---
    float mIbi = meanOf(ibis, ni), mAmp = meanOf(amp, np);
    float qRhythm = 1.0f - stdOf(ibis, ni) / fmaxf(mIbi, 1e-9f);
    float qAmp    = 1.0f - stdOf(amp, np) / fmaxf(mAmp, 1e-9f);
    // 標準化後 RMS 恆為 1,突出度就是「平均峰高等於幾個標準差」
    float qProm = mAmp / 2.5f;

    qRhythm = fmaxf(0.0f, fminf(1.0f, qRhythm));
    qAmp    = fmaxf(0.0f, fminf(1.0f, qAmp));
    qProm   = fmaxf(0.0f, fminf(1.0f, qProm));

    *outBpm = bpmv;
    *outQ = qRhythm * qAmp * qProm;
}

// 每 1 秒呼叫一次 (跑在核心 1)
static void hierUpdate()
{
    static float work[HIER_OUTER];

    portENTER_CRITICAL(&dataMux);
    bool ready = (hierFill >= HIER_OUTER);
    if (ready) {
        int h = hierHead;
        for (int i = 0; i < HIER_OUTER; i++)
            work[i] = hierBuf[(h + i) % HIER_OUTER]; // 還原成時間順序
    }
    portEXIT_CRITICAL(&dataMux);
    if (!ready) return;

    float bs[HIER_SUBS], qs[HIER_SUBS];
    int nv = 0;
    for (int pos = 0; pos + HIER_INNER <= HIER_OUTER; pos += HIER_STEP) {
        float b, q;
        subWindow(&work[pos], HIER_INNER, &b, &q);
        if (b > 0 && q > 0) { bs[nv] = b; qs[nv] = q; nv++; }
    }
    if (nv == 0) { hierQuality = 0; return; }

    // 品質加權中位數:依 BPM 排序後,累積權重達總和一半處即為結果
    for (int i = 0; i < nv; i++)
        for (int j = i + 1; j < nv; j++)
            if (bs[j] < bs[i]) {
                float t = bs[i]; bs[i] = bs[j]; bs[j] = t;
                t = qs[i]; qs[i] = qs[j]; qs[j] = t;
            }
    float tot = 0; for (int i = 0; i < nv; i++) tot += qs[i];
    float acc = 0, pick = bs[nv - 1];
    for (int i = 0; i < nv; i++) { acc += qs[i]; if (acc >= tot / 2) { pick = bs[i]; break; } }

    hierBpm = (uint16_t)(pick + 0.5f);
    hierQuality = tot / nv;
}

// 取中位數 (n <= 5)
static float medianOf(const float *src, int n)
{
    float s[5];
    memcpy(s, src, n * sizeof(float));
    for (int i = 0; i < n; i++)
        for (int j = i + 1; j < n; j++)
            if (s[j] < s[i]) { float t = s[i]; s[i] = s[j]; s[j] = t; }
    return s[n / 2];
}

static uint16_t medianU16(const uint16_t *src, int n)
{
    uint16_t s[5];
    memcpy(s, src, n * sizeof(uint16_t));
    for (int i = 0; i < n; i++)
        for (int j = i + 1; j < n; j++)
            if (s[j] < s[i]) { uint16_t t = s[i]; s[i] = s[j]; s[j] = t; }
    return s[n / 2];
}

/**
 * 單一峰值偵測器 — 三個實例分別吃不同的「特徵訊號」
 *
 * 共用同一套判定核心:局部極大 + 自適應門檻 (近期峰值中位數 x THR_K)
 * + 自適應不應期 (中位 IBI x 0.55)。差別只在餵進來的特徵訊號:
 *   A 振幅      : 帶通後的訊號本身        — 訊號乾淨時最準
 *   C 微分積分  : 一階微分平方的移動總和  — 對基線飄移免疫
 *   D 二階微分  : 加速度脈波 (APG) 的 a 波 — 峰形銳利,晃動時較穩
 */
struct PeakDetector
{
    float peakBuf[5] = {0};
    int peakCount = 0;
    uint16_t ibiBuf[5] = {0};
    int ibiCount = 0;
    float boot = 1.0f;
    uint32_t last = 0;
    float p1 = 0, p2 = 0;

    void reset(uint32_t nowMs)
    {
        peakCount = 0; ibiCount = 0; boot = 1.0f;
        last = nowMs; p1 = p2 = 0;
    }

    uint16_t medIBI() const
    {
        if (ibiCount < 3) return 600;
        return medianU16(ibiBuf, min(ibiCount, 5));
    }

    // 回傳是否偵測到心跳;beatMs 為心跳時刻
    bool feed(float v, uint32_t nowMs, uint32_t *beatMs)
    {
        float sigLvl;
        if (peakCount >= 3) {
            sigLvl = medianOf(peakBuf, min(peakCount, 5));
        } else {
            boot = fmaxf(fabsf(v), boot * BOOT_DECAY);
            sigLvl = boot;
        }
        float thr = THR_K * sigLvl;
        uint32_t refr = max((uint32_t)250, (uint32_t)(0.55f * medIBI()));

        bool fired = false;
        bool isLocalMax = (p1 > p2) && (p1 >= v);
        uint32_t peakMs = nowMs - SAMPLE_MS; // 峰在前一筆樣本
        if (isLocalMax && p1 > thr && peakMs - last > refr) {
            uint32_t ibi = peakMs - last;
            last = peakMs;
            peakBuf[peakCount % 5] = p1;
            peakCount++;
            if (ibi >= 300 && ibi <= 2000) {
                ibiBuf[ibiCount % 5] = (uint16_t)ibi;
                ibiCount++;
            }
            *beatMs = peakMs;
            fired = true;
        }
        p2 = p1; p1 = v;
        return fired;
    }
};

static PeakDetector detAmp, detDeriv, detAccel;

// ---- 特徵訊號的中間狀態 ----
static constexpr int DERIV_WIN = (int)(0.15f * FS); // 微分積分視窗 150ms
static float derivBuf[DERIV_WIN] = {0};
static int derivIdx = 0;
static float derivSum = 0;
static float prevY = 0, prevY1 = 0, prevY2 = 0;
static Biquad accelLpf; // 二階微分後再低通,壓掉放大的高頻雜訊

/* ---- 投票 ----------------------------------------------------------------
 * 三個偵測器各自獨立判定,任一觸發就登記一票。當「不同偵測器」的票在
 * VOTE_WIN 毫秒內湊滿 VOTE_K 張,才確認為一次心跳,時刻取這些票的中位數。
 *
 * 各方法在心動週期上的觸發點不同 (二階微分最早、振幅最晚),VOTE_WIN 取
 * 150ms 足以涵蓋這個系統性偏移。
 *
 * ⚠ 實測提醒:合成訊號模擬顯示,以「心率準確度」而言投票 (平均誤差 38bpm)
 * 其實不如單一振幅法 (24bpm),更遠不如自相關法 (17.6bpm) — 因為低訊噪比時
 * 各方法會一起漏掉同一批心跳,投票濾掉偽陽性的同時也砍掉真心跳。
 * 因此畫面上的心率仍採用自相關結果,投票結果僅作為逐拍偵測 (波形標記、
 * 未來要做 HRV 分析時需要逐拍 IBI,那是自相關給不了的)。
 */
static constexpr uint32_t VOTE_WIN = 150;
static constexpr int VOTE_K = 2;
static volatile uint32_t fireCount[3] = {0, 0, 0}; // 診斷:各偵測器觸發次數
static volatile uint32_t voteCount = 0;
static uint32_t voteTime[3] = {0};
static bool voteOpen[3] = {false, false, false};

static void resetDetectors(uint32_t nowMs)
{
    detAmp.reset(nowMs); detDeriv.reset(nowMs); detAccel.reset(nowMs);
    derivIdx = 0; derivSum = 0;
    memset(derivBuf, 0, sizeof(derivBuf));
    prevY = prevY1 = prevY2 = 0;
    accelLpf.reset();
    for (int i = 0; i < 3; i++) voteOpen[i] = false;
}

// 採納一次 (投票確認的) 心跳
static void acceptBeat(uint32_t tMs)
{
    uint32_t ibi = tMs - lastBeatMs;
    lastBeatMs = tMs;
    if (ibi >= 300 && ibi <= 2000) { // 30–200 bpm 合理區間
        ibiBuf[ibiCount % 5] = (uint16_t)ibi;
        ibiCount++;
        if (ibiCount >= 3)
            bpm = 60000 / medianU16(ibiBuf, min(ibiCount, 5));
    }
}

// 跑三個偵測器並投票
static void runDetectors(float y, uint32_t nowMs)
{
    // 特徵 A:帶通訊號本身
    // 特徵 C:一階微分平方的 150ms 移動總和
    float d1 = y - prevY;
    float sq = d1 * d1;
    derivSum += sq - derivBuf[derivIdx];
    derivBuf[derivIdx] = sq;
    derivIdx = (derivIdx + 1) % DERIV_WIN;
    // 特徵 D:二階微分取負 (讓 APG 的 a 波成為正峰) 再低通
    float d2 = accelLpf.process(-(y - 2 * prevY1 + prevY2));
    prevY2 = prevY1; prevY1 = y; prevY = y;

    uint32_t bt;
    if (detAmp.feed(y, nowMs, &bt))        { voteTime[0] = bt; voteOpen[0] = true; fireCount[0]++; }
    if (detDeriv.feed(derivSum, nowMs, &bt)){ voteTime[1] = bt; voteOpen[1] = true; fireCount[1]++; }
    if (detAccel.feed(d2, nowMs, &bt))     { voteTime[2] = bt; voteOpen[2] = true; fireCount[2]++; }

    // 清掉過期的票
    for (int i = 0; i < 3; i++)
        if (voteOpen[i] && nowMs - voteTime[i] > VOTE_WIN) voteOpen[i] = false;

    // 計票
    int n = 0; uint16_t times[3];
    for (int i = 0; i < 3; i++)
        if (voteOpen[i]) times[n++] = (uint16_t)(nowMs - voteTime[i]);
    if (n >= VOTE_K) {
        // 取中位數 (以「距今多久」為單位,再換回絕對時刻)
        for (int i = 0; i < n; i++)
            for (int j = i + 1; j < n; j++)
                if (times[j] < times[i]) { uint16_t t = times[i]; times[i] = times[j]; times[j] = t; }
        uint32_t beat = nowMs - times[n / 2];
        if (beat - lastBeatMs > 250) { acceptBeat(beat); voteCount++; }
        for (int i = 0; i < 3; i++) voteOpen[i] = false;
    }
}

// ---------- 壓力辨識 (WESAD LDA,11 維時域 HRV) ----------
/* 整條鏈與 tools/extract_wesad.py 逐行對應,細節見 src/ppg_chain.h。
 *
 *   原始 100Hz → PpgChain (重取樣 64Hz → Butterworth 0.5-10 → MA3 串流)
 *   → 3840 樣本環形緩衝 (60 秒)
 *   → 每 64 樣本 (1 秒):MA3 → 中位數/MAD 正規化 → find_peaks(0.3, 26) → RR → 18 特徵 → LDA
 *
 * 視窗長度 60 秒必須與訓練時相同,否則特徵分布對不上;但「多久算一次」只影響
 * 更新頻率,每次都是拿最近 60 秒重算,所以步長可以與訓練時的 10 秒不同。
 * PC 端的 10 秒步長是為了控制訓練樣本的冗餘度 (3 秒步長會讓相鄰視窗重疊 95%)。
 *
 * 注意送進來的是「原始」綠光讀值,不是手錶顯示心率用的那條 0.7-3.5Hz 帶通輸出 ——
 * 這條鏈有自己的 Butterworth,串兩層帶通就跟訓練時不是同一回事了。
 *
 * 跑在核心 1 的 loop() 裡:3840 點尋峰要數十毫秒,放進取樣任務會毀掉取樣時序。
 */
// 視窗長度由 hrv_features.h 的 HRV_WINDOW_SEC 決定 (必須與訓練時相同);
// 步長只影響「多久算一次」,每次都是拿最近整個視窗重算,可與訓練時不同。
static constexpr int   STRESS_WIN  = HRV_WINDOW_SEC * (int)PPG_FS;   // 120s = 7680 樣本
static constexpr int   STRESS_STEP_NORMAL = (int)(1 * PPG_FS);       // 2026-10-29 使用者:「先改每秒判斷」→ 平時每秒正式判定一次 (10/29 以前是 30 秒、視窗不重疊;視窗仍是 120 秒,每秒滑動 1 秒)
static constexpr int   STRESS_STEP_CALIB  = (int)(1 * PPG_FS);       // 校正程序期間每秒一次,手機才收得到足夠分數
static int             STRESS_STEP = STRESS_STEP_NORMAL;
/* 2026-09-23 省電:狀態列從「每秒一筆」改成「每次推論後一筆」(預設 30 s) + 有變化就立刻補一筆。
 * 手錶本來就每 30 秒才推論一次,中間 29 筆是重複值 (app 的事件偵測用 (score,state,nRR) 去重,本來就看不出差別);
 * 通知次數 86,400/天 → 約 2,900/天。SRATE,<秒> 可要求加快 (手錶頁打開時 app 會送 SRATE,1);
 * 校正期間 (STRESS_STEP_CALIB) 自動每秒一筆,不必 app 另外要求。 */
static uint32_t statPeriodMs = 30000;
static volatile bool statDirty = true;   // 有新推論或 worn/充電/REC 改變 → 下一圈立刻送
/* 校正模式 (app 的個人化校正:放鬆 3 分鐘 → Stroop 3 分鐘,由手機計時):FAST,1 讓推論每秒一次,
 * FAST,0 或 15 分鐘後自動恢復 30 秒一次 (app 被殺也不會一直耗電)。快取寫 T,ms,fast1/fast0 標記。 */
static uint32_t fastUntil = 0;
static bool     calibMode = false;   // 2026-10-29 正式判定改每秒之後,STRESS_STEP 已分不出校正模式;狀態列的 fast 欄改看這個 (app 看到 fast=1 不算事件)
static void fastSet(bool on)
{
    STRESS_STEP = on ? STRESS_STEP_CALIB : STRESS_STEP_NORMAL;
    calibMode = on;
    fastUntil = on ? millis() + 15UL * 60 * 1000 : 0;
    char ln[24]; int n = snprintf(ln, sizeof(ln), "T,%lu,fast%d\n", (unsigned long)millis(), on ? 1 : 0); logPush(ln, n);
    Serial.printf("# stress step %s\n", on ? "1 s (calib)" : "1 s (normal; 30 s before 2026-10-29)");
}
static constexpr int   STRESS_MAX_PK = 512;
// 參數集中在 hrv_features.h,三個韌體與 PC 端共用同一組值
static constexpr float STRESS_PROM = HRV_PEAK_PROM;
static constexpr int   STRESS_DIST = HRV_PEAK_DIST_MS * HRV_FS / 1000;

static PpgChain stressChain;
static float   *stressBuf = nullptr;    // 120 s × 64 Hz = 30 KB,放 PSRAM (內部 RAM 要留給 240×240 畫布)
static int      stressHead = 0, stressFill = 0;
static volatile uint32_t stressCount = 0;
static float    stressDcRef = 0;
static bool     stressHaveDc = false;

static float    stressScoreVal = 0;
/* 2026-09-15:兩個模型並行。stressModel 0 = LR 逐拍 18 特徵 (原本), 1 = tiny 1D-CNN 不逐拍 (預設)。
 * score/state 用選定的模型;另一個模型的分數放狀態行第 15 欄 (alt),讓前導實驗同時評估兩者。
 * 指令 MODEL,0|1 切換並存 NVS;判定線尺度不同,切換時校正線作廢 (stressModelId 含模型)。 */
static int      stressModel = 1;
static float    stressAltVal = NAN;     // 另一個模型的分數 (NaN = 這輪算不出來)
static float    stressLrVal = NAN, stressCnnVal = NAN;
static float    stressDefaultThr() { return stressModel == 1 ? 0.0f : STRESS_CALIB_OFFSET; }
// -2 = 還沒算過 (視窗剛填滿), -1 = 算過但 RR 不足, 0 = 非壓力, 1 = 壓力
// -2 與 -1 必須分開:混用會讓第一次推論前的空檔誤顯示成「資料不足」
static int      stressState = -2;
/* 判定線:預設為 stress_calib.h 的編譯值,手機 app 校正後用 THR,<v> 覆寫並存進 NVS,重開機仍有效。 */
#include <Preferences.h>
static float stressThr = STRESS_CALIB_OFFSET;
/* 模型版本:用截距的位元樣式當識別碼,模型重新匯出就會變。NVS 裡的校正線是在舊模型分數尺度上
 * 量的,模型一換就作廢,否則會用錯尺度 (例如 LDA 30 s 的 6.5 套在 LR 120 s 上會永遠 STRESS)。 */
static uint32_t stressModelId() { uint32_t v; float b = stressModel == 1 ? CNN_BL : STRESS_B; memcpy(&v, &b, 4); return v ^ (uint32_t)(HRV_WINDOW_SEC * 2654435761u) ^ (stressModel == 1 ? 0xC7A10000u : 0u); }
static void stressThrLoad()
{
    Preferences p;
    if (p.begin("stress", false)) {
        stressModel = (int)p.getUChar("model", 1);
        uint32_t ver = p.getUInt("ver", 0);
        if (ver != stressModelId()) {
            if (p.isKey("thr")) Serial.println("# stress thr in NVS belongs to an older model -> discarded");
            p.remove("thr"); p.putUInt("ver", stressModelId());
        }
        stressThr = p.getFloat("thr", stressDefaultThr()); p.end();
    }
    Serial.printf("# stress model=%s thr=%.4f (%s)\n", stressModel == 1 ? "cnn" : "lr", stressThr, stressThr == stressDefaultThr() ? "default" : "nvs");
}
static void stressModelSet(int m)
{
    if (m != 0 && m != 1) return;
    Preferences p;
    if (p.begin("stress", false)) { p.putUChar("model", (uint8_t)m); p.end(); }
    stressModel = m;
    stressThrLoad();                       // 模型換了 → 舊校正線作廢並套預設值
    stressState = -2; stressScoreVal = 0; stressAltVal = NAN;
    Serial.printf("# stress model set %s\n", m == 1 ? "cnn (tiny 1D-CNN, no beats)" : "lr (18 HRV features)");
}
static void stressThrSet(float v, bool reset)
{
    Preferences p;
    if (p.begin("stress", false)) { if (reset) p.remove("thr"); else p.putFloat("thr", v); p.end(); }
    stressThr = reset ? stressDefaultThr() : v;
    Serial.printf("# stress thr set %.4f%s\n", stressThr, reset ? " (reset)" : "");
}
static int      stressNPeak = 0, stressNRR = 0;
static uint32_t stressUs = 0;
/* 2026-10-29 使用者:「每秒更新一次壓力狀態」→ 時鐘頁用的即時分數:每秒對目前的 120 秒視窗跑一次選定的模型,只給螢幕顯示。
 * 正式判定 (stressScoreVal／stressState:給 app 的狀態列與事件偵測) 仍每 30 秒一次,FAST 校正模式也不變,所以 app「連續 3 次判定
 * 才算事件」的語意沒動。LIVE,0 可關掉 (關掉時時鐘頁顯示最近一次正式判定)。CNN 一次約 45 ms,在核心 1 的主迴圈跑,不影響取樣。 */
static bool     liveOn = true;
static float    liveScore = NAN;        // 即時分數 (NaN = 還沒算過、這一秒算不出來、或未配戴)
static int      liveState = -2;         // 意義同 stressState
static uint32_t stressJudgeN = 0, stressJudgeMs = 0;   // 正式判定的次數與最近一次的 millis (時鐘頁顯示「幾秒前」)

/* 清空壓力視窗。接觸閘門移除後平時不會被呼叫,保留給重置與偵錯用。 */
static void stressReset()
{
    stressChain.reset();
    stressHead = 0; stressFill = 0; stressCount = 0;
    stressState = -2; stressScoreVal = 0; stressNPeak = 0; stressNRR = 0;
    stressHaveDc = false;
    liveScore = NAN; liveState = -2;
}

static void stressSink(float v)
{
#if BLE_ENABLED
    ble::pushPpg(v);
#endif
    stressBuf[stressHead] = v;
    stressHead = (stressHead + 1) % STRESS_WIN;
    if (stressFill < STRESS_WIN) stressFill++;
    stressCount++;
}

/* 餵一筆原始 100Hz 讀值。先扣掉進場時的直流參考:帶通穩態下本來就會濾掉常數,
 * 扣掉只是讓 float32 不必在 25000 的量級上表示 200 的脈動,不改變穩態輸出。 */
static void stressFeed(float raw)
{
    if (!stressHaveDc) { stressDcRef = raw; stressHaveDc = true; }
    stressChain.push(raw - stressDcRef, stressSink);
}


static float *stressWork = nullptr;    // 視窗運算暫存 (PSRAM)
// ACC 遮罩:20 Hz 三軸環形緩衝 (120 s) 與 64 Hz 對齊後的幀資料、STFT 暫存,全在 PSRAM
static constexpr int ACC_FS = 20, ACC_RING = HRV_WINDOW_SEC * ACC_FS;
static float   (*accRing)[3] = nullptr; static int accHead = 0, accFill = 0;
static float   (*acc64)[3] = nullptr;    // STRESS_WIN × 3
static float   *mExt = nullptr, *mOut = nullptr, *mNorm = nullptr;
static constexpr int MASK_TMP = STRESS_WIN + 2 * stftmask::NPER;
static bool     maskEnabled = false;   // 預設關:DaLiA 走路有改善,但 BAMI-1 與 PhysioNet wrist 兩組沒有,證據不足 (2026-09-10)。序列或 app 指令 MASK,1 可開
static void accPush(float ax, float ay, float az)   // m/s² → g
{
    if (!accRing) return;
    accRing[accHead][0] = ax / 9.80665f; accRing[accHead][1] = ay / 9.80665f; accRing[accHead][2] = az / 9.80665f;
    accHead = (accHead + 1) % ACC_RING; if (accFill < ACC_RING) accFill++;
}
static float stressFeat[STRESS_N_FEAT]; // 最近一次的完整特徵向量 (供輸出比對)

/* 對一段已帶通並平滑過的 30 秒訊號做:MA3 -> 中位數/MAD 正規化 -> 尋峰 -> RR -> 特徵 -> LDA。
 * w 會被就地修改。實際量測與開機自檢共用這一段,確保驗證到的就是實際跑的程式碼。 */
static bool stressComputeWindow(float *w, int n, float *outScore, HrvResult *hrv)
{
    if (maskEnabled && accFill >= ACC_RING && acc64 && mExt) {
        // 20 Hz 環形緩衝 → 64 Hz 線性內插,兩者都以「現在」為視窗尾端對齊
        for (int i = 0; i < n; i++) {
            float pos = (float)i * ACC_FS / HRV_FS;
            int k = (int)pos; float fr = pos - k; if (k >= ACC_RING - 1) { k = ACC_RING - 2; fr = 1.0f; }
            int i0 = (accHead + k) % ACC_RING, i1 = (accHead + k + 1) % ACC_RING;
            for (int a = 0; a < 3; a++) acc64[i][a] = accRing[i0][a] + fr * (accRing[i1][a] - accRing[i0][a]);
        }
        stftmask::apply(w, acc64, n, mExt, mOut, mNorm);
    }
    ppgSmoothInPlace(w, n);                       // 每個視窗再平滑一次 (照 PC 端)

    static float *scratch = (float *)ps_malloc(sizeof(float) * STRESS_WIN);
    if (!hrvNormalizeMad(w, n, scratch)) return false;   // 中位數/MAD 正規化 (照 PC 端)

    static int   pkIdx[STRESS_MAX_PK];
    static float pkAmp[STRESS_MAX_PK];
    int np = findPeaks(w, n, STRESS_PROM, STRESS_DIST, pkIdx, pkAmp, STRESS_MAX_PK);
    stressNPeak = np;

    if (!hrvFromPeaks(pkIdx, np, PPG_FS, hrv)) return false;

    // 特徵順序必須與 stress_model.h 的 STRESS_FEAT_NAMES 相同:
    // 18 個 HRV (時域 + 頻域) 之後接 6 個加速度特徵
    memcpy(stressFeat, hrv->feat, sizeof(float) * STRESS_N_FEAT);
    *outScore = stressInfer(stressFeat);
    return true;
}

static void stressUpdate()
{
    if (stressFill < STRESS_WIN) return;
    if (stressCount < (uint32_t)STRESS_STEP) return;
    stressCount -= (uint32_t)STRESS_STEP;   // 2026-10-29 改每秒判定後:只扣掉這一步、保留餘數。hub 每 500 ms 一批,主迴圈每秒檢查一次,相位對不上時
                                            // 原本的「歸零」會把多出來的半批丟掉,實測判定間隔變成 1、2、1、2 秒交替;保留餘數後平均每秒一次
    stressJudgeN++; stressJudgeMs = millis();

    uint32_t t0 = micros();
    // 只快照 head,複製不上鎖:複製期間取樣任務最多再寫入數筆,影響的是視窗最舊的
    // 幾個樣本,對 60 秒的統計量無關緊要;整段關中斷反而會毀掉取樣時序。
    int h = stressHead;
    for (int i = 0; i < STRESS_WIN; i++) stressWork[i] = stressBuf[(h + i) % STRESS_WIN];

    static HrvResult hrv;
    // tiny CNN 先算 (stressComputeWindow 會就地改寫 stressWork)
    float cnnLg = NAN; bool cnnOk = cnnInfer(stressWork, STRESS_WIN, &cnnLg);
    uint32_t tCnn = micros() - t0;
    float lrScore = NAN; bool lrOk = stressComputeWindow(stressWork, STRESS_WIN, &lrScore, &hrv);
    stressLrVal = lrOk ? lrScore : NAN; stressCnnVal = cnnOk ? cnnLg : NAN;
    stressNRR = hrv.nRR; wearLastNRR = hrv.nRR; wearLastNRRms = millis();
    bool ok = stressModel == 1 ? cnnOk : lrOk;
    float score = stressModel == 1 ? cnnLg : lrScore;
    stressAltVal = stressModel == 1 ? stressLrVal : stressCnnVal;
    if (!ok) {
        stressState = -1;              // 算過了,但 (LR) RR 數不足 / (CNN) 訊號平坦
        statDirty = true;
        stressUs = micros() - t0;
        Serial.printf("# stress reject model=%s nPeak=%d nRR=%d cnn=%s\n", stressModel == 1 ? "cnn" : "lr",
                      stressNPeak, hrv.nRR, cnnOk ? "ok" : "flat");
        return;
    }

    stressScoreVal = score;
    stressState = score > stressThr ? 1 : 0;
    statDirty = true;
    stressUs = micros() - t0;

    if (!serialQuiet) Serial.printf("# stress model=%s score=%.6f thr=%.4f state=%d lr=%.4f cnn=%.4f nPeak=%d nRR=%d us=%lu (cnn %lu)\n",
                  stressModel == 1 ? "cnn" : "lr", score, stressThr, stressState, stressLrVal, stressCnnVal, stressNPeak, hrv.nRR,
                  (unsigned long)stressUs, (unsigned long)tCnn);
    Serial.print("# feat");
    for (int i = 0; i < STRESS_N_FEAT; i++) Serial.printf(",%.8g", stressFeat[i]);
    Serial.println();
    Serial.print("# rr");
    for (int i = 0; i < hrv.nRR; i++) Serial.printf(",%.4f", hrv.rr[i]);
    Serial.println();
}

/* 2026-10-29 即時分數 (只給時鐘頁):每秒跑一次選定的模型;這一秒剛做過正式判定就直接沿用,不多算一次。
 * 不印序列埠、不寫快取、不送狀態列。LIVE,0 時只是照抄正式判定。 */
static void stressLiveUpdate(bool justJudged)
{
    if (!liveOn || justJudged) { liveScore = stressState >= 0 ? stressScoreVal : NAN; liveState = stressState; return; }
    if (stressFill < STRESS_WIN || !stressWork) { liveScore = NAN; liveState = -2; return; }
    int h = stressHead;
    for (int i = 0; i < STRESS_WIN; i++) stressWork[i] = stressBuf[(h + i) % STRESS_WIN];
    float sc = NAN; bool ok;
    if (stressModel == 1) ok = cnnInfer(stressWork, STRESS_WIN, &sc);
    else { static HrvResult hrvLive; ok = stressComputeWindow(stressWork, STRESS_WIN, &sc, &hrvLive); }
    if (!ok) { liveScore = NAN; liveState = -1; return; }
    liveScore = sc; liveState = sc > stressThr ? 1 : 0;
}

/* 開機自檢:用一段確定性的 100Hz 合成訊號跑完整條鏈 —— 重取樣 → Butterworth
 * → MA3 → 視窗運算 → 特徵 → LDA,印出結果。tools/verify_chain.py 在 PC 端用
 * 同一條產生規則與同一組 scipy 函式重算後比對。
 * 刻意從 100Hz 進入,才能一併驗證重取樣:那一段若有偏差是系統性的
 * (所有 RR 等比例偏移),不會在特徵公式的比對裡顯現。 */
static constexpr int STRESS_TEST_SKIP = 512;   // 前 512 筆輸出留給濾波器穩定
/* 遮罩自檢:固定的合成 PPG (1.6 Hz + 2 倍頻 + 1.9 Hz 步頻偽跡) 與合成 ACC (1.9 Hz),
 * 跑 stftmask::apply 後印前幾個樣本與總和;tools/verify_mask.py 用 stft_mask_ref 算同一組數字比對。 */
static void maskSelfTest()
{
    const int n = 1920;   // 30 s 就夠驗證
    float *x = (float *)ps_malloc(sizeof(float) * n);
    float (*a)[3] = (float (*)[3])ps_malloc(sizeof(float) * 3 * n);
    if (!x || !a || !mExt) { Serial.println("# masktest skipped (no PSRAM)"); return; }
    for (int i = 0; i < n; i++) {
        float t = (float)i / 64.0f;
        x[i] = sinf(2 * (float)M_PI * 1.6f * t) + 0.3f * sinf(2 * (float)M_PI * 3.2f * t) + 2.5f * sinf(2 * (float)M_PI * 1.9f * t + 0.3f);
        a[i][0] = 0.5f * sinf(2 * (float)M_PI * 1.9f * t); a[i][1] = 0.5f * sinf(2 * (float)M_PI * 1.9f * t + 1.0f); a[i][2] = 1.0f + 0.5f * sinf(2 * (float)M_PI * 1.9f * t + 2.0f);
    }
    stftmask::apply(x, a, n, mExt, mOut, mNorm);
    double sum = 0, sq = 0; for (int i = 0; i < n; i++) { sum += x[i]; sq += (double)x[i] * x[i]; }
    Serial.printf("# masktest sum=%.6f sq=%.6f s100=%.6f s500=%.6f s1000=%.6f s1500=%.6f\n", sum, sq, x[100], x[500], x[1000], x[1500]);
    free(x); free(a);
}

static void stressSelfTest()
{
    PpgChain ch; ch.reset();
    int nOut = 0;
    const int NIN = (STRESS_TEST_SKIP + STRESS_WIN + 8) * 100 / 64 + 64;  // 100Hz 輸入
    for (int j = 0; j < NIN && nOut < STRESS_TEST_SKIP + STRESS_WIN; j++) {
        double t = (double)j / 100.0;
        double amp = 1.0 + 0.25 * sin(2.0 * M_PI * 0.07 * t);
        double x = amp * sin(2.0 * M_PI * 1.15 * t)
                 + 0.35 * sin(2.0 * M_PI * 2.30 * t + 1.0)
                 + 0.40 * sin(2.0 * M_PI * 0.05 * t)
                 + 0.03 * sin(2.0 * M_PI * 11.0 * t);
        ch.push((float)x, [&](float v) {
            int k = nOut - STRESS_TEST_SKIP;
            if (k >= 0 && k < STRESS_WIN) stressWork[k] = v;
            nOut++;
        });
    }
    if (nOut < STRESS_TEST_SKIP + STRESS_WIN) {
        Serial.printf("# chaintest SHORT nOut=%d\n", nOut);
        return;
    }

    HrvResult hrv; float score;
    if (!stressComputeWindow(stressWork, STRESS_WIN, &score, &hrv)) {
        Serial.printf("# chaintest REJECT nPeak=%d nRR=%d\n", stressNPeak, hrv.nRR);
        return;
    }
    Serial.printf("# chaintest nPeak=%d nRR=%d score=%.8g\n",
                  stressNPeak, hrv.nRR, score);
    Serial.print("# chaintest_feat");
    for (int i = 0; i < HRV_N_FEAT; i++) Serial.printf(",%.9g", hrv.feat[i]);
    Serial.println();
}

// ---------- 活動辨識 (WISDM 手錶 ACC+陀螺儀, LDA, LOSO 88.2%) ----------
/* IMU 在核心 1 的 loop() 以 micros 排程每 50ms 讀一次 (20 Hz),與觸控同執行緒;
 * 實測從核心 0 讀會與 LovyanGFX 的 I2C 驅動衝突 (Error 263) 並拖垮 PPG 取樣。
 * 特徵與推論每秒做一次。 */
static float harAcc[HAR_WIN][3], harGyr[HAR_WIN][3];
static volatile int harHead = 0, harFill = 0;
static volatile uint32_t harMiss = 0;
static int   harClass = -1;               // -1 = 尚無結果
static float harScores[HAR_N_CLASS];
static float harFeat[HAR_N_FEAT];

static bool imuFifo = true;               // IMUFIFO,0|1:true = 每 200 ms 讀 FIFO 再抽成 20 Hz;false = 每 50 ms 輪詢一筆
static uint32_t imuFifoFail = 0;
static void imuFifoSet(bool on)
{
    pmBusEnter();
    if (on) on = imu.fifoBegin(); else imu.fifoEnd();
    pmBusExit();
    imuFifo = on; imuFifoFail = 0;
    Serial.printf("# IMU %s\n", on ? "FIFO batch (200 ms)" : "poll 20 Hz");
}
static void harPush(const float *a, const float *g, const int16_t *rawA, const int16_t *rawG, uint32_t tms)
{
    int h = harHead;
    memcpy(harAcc[h], a, sizeof(float) * 3); memcpy(harGyr[h], g, sizeof(float) * 3);
    // 原始 IMU 一併輸出 (20 行/秒),離線分析動作偽跡時要和 PPG 同步
    {
        char ln[80]; int n = (imu.ctrl7 & 0x02)
            ? snprintf(ln, sizeof(ln), "I,%lu,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f\n", (unsigned long)tms, a[0], a[1], a[2], g[0], g[1], g[2])
            : snprintf(ln, sizeof(ln), "I,%lu,%.3f,%.3f,%.3f\n", (unsigned long)tms, a[0], a[1], a[2]);   // 陀螺儀關掉就不寫那三欄
        if (liveSerial) Serial.write((const uint8_t *)ln, n);
        if (logFmt != 1) logPush(ln, n);
        if (logFmt != 0) logImuBin(tms, rawA, rawG);
    }
#if BLE_ENABLED
    ble::pushImu(a[0], a[1], a[2], millis());
#endif
#if STRESS_ENABLED
    accPush(a[0], a[1], a[2]);
#endif
    harHead = (h + 1) % HAR_WIN;
    if (harFill < HAR_WIN) harFill = harFill + 1;
}
static void harSample()                   // 輪詢模式:每 50 ms 讀一筆
{
    if (!imu.ok) return;
    float a[3], g[3];
    pmBusEnter(); bool ok = imu.read(a, g); pmBusExit();
    if (!ok) { harMiss++; return; }
    harPush(a, g, imu.rawA, imu.rawG, millis());
}
static void harFifoPoll()                 // FIFO 模式:每 200 ms 把 FIFO 讀空,依時間抽成 20 Hz (每 50 ms 取最接近的一筆)
{
    if (!imu.ok) return;
    static uint32_t nextMs = 0;
    uint32_t tRead = millis();
    struct S { float a[3], g[3]; int16_t ra[3], rg[3]; };
    static S buf[64]; int n = 0;
    pmBusEnter();
    int got = imu.fifoRead([&](const float *a, const float *g, const int16_t *ra, const int16_t *rg) {
        if (n < 64) { memcpy(buf[n].a, a, 12); memcpy(buf[n].g, g, 12); memcpy(buf[n].ra, ra, 6); memcpy(buf[n].rg, rg, 6); n++; }
    });
    pmBusExit();
    if (got < 0) {
        harMiss++;
        if (++imuFifoFail >= 10) { Serial.println("# IMU FIFO failing, fall back to poll"); imuFifoSet(false); }
        return;
    }
    imuFifoFail = 0;
    if (n == 0) return;
    const float dt = 1000.0f / QMI_ODR_HZ;
    if (nextMs == 0 || (int32_t)(tRead - nextMs) > 1000) nextMs = tRead - (uint32_t)((n - 1) * dt);   // 第一次/落後太多:重新對齊
    for (int k = 0; k < n; k++) {
        uint32_t tk = tRead - (uint32_t)((n - 1 - k) * dt);   // 這筆的時間往回推
        if ((int32_t)(tk - nextMs) >= 0) { harPush(buf[k].a, buf[k].g, buf[k].ra, buf[k].rg, tk); nextMs += 50; }
    }
}

static void harUpdate()
{
    if (harFill < HAR_WIN) return;
    static float acc[HAR_WIN][3], gyr[HAR_WIN][3];
    int h = harHead;
    for (int i = 0; i < HAR_WIN; i++) {
        int k = (h + i) % HAR_WIN;
        memcpy(acc[i], harAcc[k], sizeof(acc[i])); memcpy(gyr[i], harGyr[k], sizeof(gyr[i]));
    }
    harFeatures(acc, gyr, HAR_WIN, harFeat);
    {   // 配戴偵測用:|a| 的標準差
        float m[HAR_WIN], mu = 0;
        for (int i = 0; i < HAR_WIN; i++) { m[i] = sqrtf(acc[i][0]*acc[i][0] + acc[i][1]*acc[i][1] + acc[i][2]*acc[i][2]); mu += m[i]; }
        mu /= HAR_WIN; float v = 0;
        for (int i = 0; i < HAR_WIN; i++) v += (m[i] - mu) * (m[i] - mu);
        wearAccStd = sqrtf(v / HAR_WIN);
        if (wearAccStd > WEAR_MOTION_STD) { lastMotionMs = millis(); wearMotionEver = true; }
    }
    float prob[HAR_N_CLASS];
    uint32_t t0 = micros();
    harClass = harForestInfer(harFeat, prob);           // 判定用隨機森林
    uint32_t us = micros() - t0;
    if (logRec) {   // 活動判定也進錄音,離線分析才能自動分段
        char ln[24]; int n = snprintf(ln, sizeof(ln), "H,%lu,%s\n", (unsigned long)millis(), HAR_CLASS_NAMES[harClass]);
        logPush(ln, n);
    }
    int lda = harInfer(harFeat, harScores);            // LDA 只當診斷對照
    if (!serialQuiet) Serial.printf("# har class=%s prob=%.2f,%.2f,%.2f,%.2f lda=%s us=%lu miss=%lu\n",
                  HAR_CLASS_NAMES[harClass], prob[0]/HARF_N_TREE, prob[1]/HARF_N_TREE,
                  prob[2]/HARF_N_TREE, prob[3]/HARF_N_TREE, HAR_CLASS_NAMES[lda],
                  (unsigned long)us, (unsigned long)harMiss);
}

/* 開機自檢:確定性的合成 IMU 視窗 → 特徵 → 分數,tools/verify_har.py 在 PC 端重算比對。 */
static void harSelfTest()
{
    static float acc[HAR_WIN][3], gyr[HAR_WIN][3];
    for (int i = 0; i < HAR_WIN; i++) {
        double t = (double)i / HAR_FS;
        acc[i][0] = (float)(1.5 * sin(2*M_PI*1.8*t));
        acc[i][1] = (float)(9.81 + 2.0 * sin(2*M_PI*1.8*t + 0.7));
        acc[i][2] = (float)(-0.8 + 0.9 * sin(2*M_PI*3.6*t));
        gyr[i][0] = (float)(0.6 * sin(2*M_PI*1.8*t + 1.1));
        gyr[i][1] = (float)(0.2 * sin(2*M_PI*0.9*t));
        gyr[i][2] = (float)(-0.3 + 0.4 * sin(2*M_PI*1.8*t + 2.0));
    }
    float f[HAR_N_FEAT], sc[HAR_N_CLASS];
    harFeatures(acc, gyr, HAR_WIN, f);
    int c = harInfer(f, sc);
    float pr[HAR_N_CLASS];
    int cf = harForestInfer(f, pr);
    Serial.printf("# hartest class=%d forest=%d\n", c, cf);
    Serial.print("# hartest_feat");
    for (int i = 0; i < HAR_N_FEAT; i++) Serial.printf(",%.9g", f[i]);
    Serial.println();
    Serial.print("# hartest_score");
    for (int i = 0; i < HAR_N_CLASS; i++) Serial.printf(",%.9g", sc[i]);
    Serial.println();
}

// ---------- 板上錄音:PPG + IMU 寫進 flash,事後由序列埠倒出 ----------
/* 插著 USB 線沒辦法走路,所以錄到 3.4 MB 的 spiffs 分割區 (約 8 分鐘)。
 * 生產端 (核心 0 的 PPG 任務、核心 1 的 IMU 取樣) 只往 RAM 環形緩衝丟字串;
 * 寫 flash 由核心 1 的 loop() 每 2 秒批次做,避免 flash 寫入時的快取停頓拖累取樣。
 *
 * 控制:觸控長按 1.5 秒切換錄製;或序列埠指令 REC / STOP / DUMP / CLEAR。
 * 格式與序列埠即時輸出相同:P,ms,green  與  I,ms,ax,ay,az,gx,gy,gz */
static constexpr size_t LOG_RING = 32768;
static char     logRing[LOG_RING];
static volatile size_t logHead = 0, logTail = 0;
static bool     logFsOk = false;
static uint32_t logStartMs = 0, logBytes = 0, logDropped = 0;
static File     logFile;

/* P 行由核心 0 的取樣任務推入,I/H/T/C 行由核心 1 的 loop 推入:兩邊同時寫環形緩衝
 * 會把兩行絞在一起 (實測 99 秒出現 3 次半截行),所以整段用自旋鎖包起來。 */
static portMUX_TYPE logMux = portMUX_INITIALIZER_UNLOCKED;
static void logPush(const char *s, size_t n)
{
    if (!logRec) return;
    portENTER_CRITICAL(&logMux);
    size_t head = logHead, tail = logTail;
    size_t used = (head + LOG_RING - tail) % LOG_RING;
    if (used + n + 1 >= LOG_RING) { logDropped++; portEXIT_CRITICAL(&logMux); return; }
    for (size_t i = 0; i < n; i++) { logRing[head] = s[i]; head = (head + 1) % LOG_RING; }
    logHead = head;
    portEXIT_CRITICAL(&logMux);
}

/* ---------- 無損二進位快取 (2026-09-14) ----------
 * 檔案仍是位元組串流:以 ASCII 字母開頭的是文字行 (B/H/T/W/Z/F,到 '\n' 為止);第一個位元組 >= 0x80 的是二進位紀錄。
 *   0xF0 PPG 區塊:[0xF0][n u8][ms0 u32][g0 u32][ir0 u32] 接著 (n-1) 組 (dg i16, dir i16);第 k 筆的 ms = ms0 + 10k。
 *        差值超出 ±32767 時寫 i16 = -32768 再接完整 u32。時間戳不是 +10 ms 或區塊滿 25 筆就另起區塊。
 *   0xF1 IMU 紀錄:[0xF1][ms u32][ax ay az gx gy gz 各 i16 原始計數],換算常數見 qmi8658.h。
 * 每個 30 秒檔開頭寫文字行 "F,<fmt>,<ms>"。解碼器 tools/cachefmt.py (PC) 與 app/lib/cachefmt.dart (手機) 可還原成逐筆相同的 P/I 行。 */
static constexpr int LOG_BLK_N = 25;
static uint8_t  blkBuf[2 + 4 + 8 + (LOG_BLK_N - 1) * 12]; static int blkLen = 0, blkN = 0;
static uint32_t blkPrevMs = 0, blkPrevG = 0, blkPrevIr = 0;
static inline void put16(uint8_t *b, int16_t v) { b[0] = (uint8_t)(v & 0xFF); b[1] = (uint8_t)((v >> 8) & 0xFF); }
static inline void put32(uint8_t *b, uint32_t v) { b[0] = v & 0xFF; b[1] = (v >> 8) & 0xFF; b[2] = (v >> 16) & 0xFF; b[3] = (v >> 24) & 0xFF; }
static void blkFlush() { if (blkN > 0) { blkBuf[1] = (uint8_t)blkN; logPush((const char *)blkBuf, blkLen); } blkN = 0; blkLen = 0; }
static int  putDelta(uint8_t *b, uint32_t cur, uint32_t prev)
{
    int64_t d = (int64_t)cur - (int64_t)prev;
    if (d >= -32767 && d <= 32767) { put16(b, (int16_t)d); return 2; }
    put16(b, (int16_t)-32768); put32(b + 2, cur); return 6;
}
static void logPpgBin(uint32_t ms, uint32_t green, uint32_t ir)
{
    if (blkN > 0 && (ms != blkPrevMs + SAMPLE_MS || blkN >= LOG_BLK_N)) blkFlush();
    if (blkN == 0) { blkBuf[0] = 0xF0; put32(blkBuf + 2, ms); put32(blkBuf + 6, green); put32(blkBuf + 10, ir); blkLen = 14; }
    else { blkLen += putDelta(blkBuf + blkLen, green, blkPrevG); blkLen += putDelta(blkBuf + blkLen, ir, blkPrevIr); }
    blkN++; blkPrevMs = ms; blkPrevG = green; blkPrevIr = ir;
}
static void logImuBin(uint32_t ms, const int16_t *a, const int16_t *g)
{
    if (imu.ctrl7 & 0x02) {                     // 陀螺儀開著 (GYRO,1):舊格式 0xF1,6 軸
        uint8_t r[17]; r[0] = 0xF1; put32(r + 1, ms);
        for (int i = 0; i < 3; i++) { put16(r + 5 + 2 * i, a[i]); put16(r + 11 + 2 * i, g[i]); }
        logPush((const char *)r, sizeof(r));
    } else {                                    // 2026-09-18 預設只有加速度:0xF2 = [ms u32][ax ay az i16],11 bytes
        uint8_t r[11]; r[0] = 0xF2; put32(r + 1, ms);
        for (int i = 0; i < 3; i++) put16(r + 5 + 2 * i, a[i]);
        logPush((const char *)r, sizeof(r));
    }
}

/* 2026-09-10 改為「常駐 10 分鐘快取」:開機後永遠在錄,flash 上以 30 秒一個檔 (/cNNNN.csv) 輪替,
 * 只留最新的 LOG_KEEP 個完成檔 + 正在寫的檔 (約 10.5 分鐘)。REC/STOP 只寫段落標記,DUMP / GET 回傳
 * 所有檔依序串接的內容 (最舊 → 最新)。手機 app 被關掉時,最近 10 分鐘的原始訊號仍抓得回來。 */
static constexpr uint32_t LOG_CHUNK_MS = 30000;
static constexpr int      LOG_KEEP     = 20;              // 20 × 30 s = 10 分鐘 (每檔約 95 KB,共約 2 MB)
static uint32_t logSeq = 0, logChunkStart = 0;
/* 讀取快照:DUMP 時把檔案清單與大小定住,手機分段 GET 期間即使檔案輪替,偏移量也不漂移;
 * 快照內的檔在 SNAP_HOLD_MS 內不被刪 (最多多留 6 個,約 0.6 MB,分割區 3.4 MB 夠用)。 */
static uint32_t snapSeqs[LOG_KEEP + 2], snapSizes[LOG_KEEP + 2]; static int snapN = 0;
static uint32_t snapTotal = 0, snapMs = 0; static bool snapActive = false;
static constexpr uint32_t SNAP_HOLD_MS = 10 * 60 * 1000UL;
static File snapFile; static int snapK = -1;
static bool snapHold() { return snapActive && (millis() - snapMs < SNAP_HOLD_MS) && (logSeq - snapSeqs[0] <= (uint32_t)(LOG_KEEP + 6)); }
static char     logPath[24];

static void logChunkName(char *out, size_t cap, uint32_t seq) { snprintf(out, cap, "/c%04lu.csv", (unsigned long)seq); }

static void logPrune()
{   // 刪掉比 (logSeq - LOG_KEEP) 更舊的檔
    File dir = LittleFS.open("/");
    if (!dir) return;
    for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
        const char *n = f.name(); unsigned long q;
        if (sscanf(n, "c%lu.csv", &q) == 1 || sscanf(n, "/c%lu.csv", &q) == 1) {
            if (q + LOG_KEEP < logSeq && !(snapHold() && q >= snapSeqs[0])) { char p[24]; logChunkName(p, sizeof(p), q); f.close(); LittleFS.remove(p); continue; }
        }
        f.close();
    }
}

static void logOpenChunk()
{
    if (logFile) logFile.close();
    logChunkName(logPath, sizeof(logPath), logSeq);
    logFile = LittleFS.open(logPath, FILE_WRITE);
    logChunkStart = millis();
    if (logFile) { char h[32]; int n = snprintf(h, sizeof(h), "F,%d,%lu\n", (int)logFmt, (unsigned long)millis()); logBytes += logFile.write((const uint8_t *)h, n); }
    logPrune();
}

static void logFlush(bool force = false)
{
    if (!logFsOk || !logFile) return;
    size_t head = logHead, tail = logTail;
    size_t used = (head + LOG_RING - tail) % LOG_RING;
    if (!force && used < 16384 && millis() - logChunkStart < LOG_CHUNK_MS) return;
    while (tail != head) {
        size_t chunk = (head > tail) ? head - tail : LOG_RING - tail;
        logBytes += logFile.write((const uint8_t *)&logRing[tail], chunk);
        tail = (tail + chunk) % LOG_RING;
    }
    logTail = tail;
    logFile.flush();
    if (millis() - logChunkStart >= LOG_CHUNK_MS) { logSeq++; logOpenChunk(); }   // 輪替到下一個 30 秒檔
}

static void logBegin()          // 開機:接續最大的序號之後,開始常駐錄製
{
    if (!logFsOk) return;
    uint32_t maxSeq = 0; bool any = false;
    File dir = LittleFS.open("/");
    if (dir) for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
        unsigned long q; const char *n = f.name();
        if (sscanf(n, "c%lu.csv", &q) == 1 || sscanf(n, "/c%lu.csv", &q) == 1) { if (!any || q > maxSeq) maxSeq = q; any = true; }
        f.close();
    }
    LittleFS.remove("/log.csv");                       // 舊格式
    logSeq = any ? maxSeq + 1 : 0;
    logHead = logTail = 0; logBytes = 0; logDropped = 0; logStartMs = millis();
    logOpenChunk();
    logRec = true;
    Serial.printf("# LOG cache on (chunk %lu, keep %d x %lu s)\n", (unsigned long)logSeq, LOG_KEEP, (unsigned long)(LOG_CHUNK_MS / 1000));
}

static void logStart()          // 相容舊指令:現在只是寫標記
{
    char ln[24]; int n = snprintf(ln, sizeof(ln), "T,%lu,rec\n", (unsigned long)millis()); logPush(ln, n);
    logMarked = true; logMarkMs = millis(); statDirty = true;
    Serial.println("# LOG rec marker");
}

static void logStop()
{
    char ln[24]; int n = snprintf(ln, sizeof(ln), "T,%lu,stop\n", (unsigned long)millis()); logPush(ln, n);
    logMarked = false; statDirty = true;
    logFlush(true);
    Serial.printf("# LOG stop marker, cache bytes=%lu dropped=%lu free=%lu\n", (unsigned long)logBytes, (unsigned long)logDropped,
                  (unsigned long)(LittleFS.totalBytes() - LittleFS.usedBytes()));
}

/* 串接讀取:所有快取檔依序號排序 (最舊 → 最新,含正在寫的) */
static int logChunks(uint32_t *seqs, int cap)
{
    int n = 0; uint32_t lo = logSeq >= (uint32_t)LOG_KEEP ? logSeq - LOG_KEEP : 0;
    for (uint32_t q = lo; q <= logSeq && n < cap; q++) { char p[24]; logChunkName(p, sizeof(p), q); if (LittleFS.exists(p)) seqs[n++] = q; }
    return n;
}
static void logSnapshot()
{
    logFlush(true);                   // 先把未寫出的緩衝落地,快照才含最近的樣本 (最多差一個未滿的 PPG 區塊 ≤ 250 ms)
    if (snapFile) snapFile.close();
    snapK = -1; snapN = logChunks(snapSeqs, LOG_KEEP + 2); snapTotal = 0;
    for (int k = 0; k < snapN; k++) {
        char p[24]; logChunkName(p, sizeof(p), snapSeqs[k]); File f = LittleFS.open(p, FILE_READ);
        snapSizes[k] = f ? f.size() : 0; if (f) f.close(); snapTotal += snapSizes[k];
    }
    snapActive = snapN > 0; snapMs = millis();
}
static void logSnapDrop() { if (snapFile) snapFile.close(); snapK = -1; snapActive = false; snapN = 0; snapTotal = 0; }
/* 增量快照 (2026-09-15,手機定期同步用):只拍「序號 > after 且已寫完」的檔 (正在寫的那個不算);
 * after >= 目前序號 (手機不知道或手錶重開機序號變小) 就視為全部。rotate=1 先把正在寫的檔強制收尾,
 * 會話結束時用,讓最後不足 30 秒的資料也拿得到。回傳 lo/hi = 快照內最小/最大序號 (無檔時皆 0)。 */
static void logSnapshotRange(uint32_t after, bool rotate, uint32_t *lo, uint32_t *hi)
{
    logFlush(true);
    if (rotate && logFsOk && logFile) { logSeq++; logOpenChunk(); }
    if (snapFile) snapFile.close();
    uint32_t all[LOG_KEEP + 2]; int n = logChunks(all, LOG_KEEP + 2);
    bool everything = after >= logSeq;
    snapK = -1; snapN = 0; snapTotal = 0; *lo = *hi = 0;
    for (int k = 0; k < n; k++) {
        uint32_t q = all[k];
        if (q >= logSeq || (!everything && q <= after)) continue;
        char p[24]; logChunkName(p, sizeof(p), q); File f = LittleFS.open(p, FILE_READ);
        uint32_t sz = f ? f.size() : 0; if (f) f.close();
        if (!sz) continue;
        if (!snapN) *lo = q;
        *hi = q; snapSeqs[snapN] = q; snapSizes[snapN] = sz; snapTotal += sz; snapN++;
    }
    snapActive = snapN > 0; snapMs = millis();
}
static uint32_t logTotalBytes()          // GET 用:沿用快照 (太舊才重拍)
{
    if (!snapActive || millis() - snapMs > SNAP_HOLD_MS) logSnapshot();
    return snapTotal;
}
static size_t logReadAt(uint32_t off, uint8_t *buf, size_t len)   // 跨檔讀取 (快照內,循序時不重開檔)
{
    if (!snapActive) logSnapshot();
    size_t got = 0; uint32_t base = 0; int k = 0;
    while (k < snapN && off >= base + snapSizes[k]) { base += snapSizes[k]; k++; }
    while (got < len && k < snapN) {
        if (snapK != k) {
            if (snapFile) snapFile.close();
            char p[24]; logChunkName(p, sizeof(p), snapSeqs[k]); snapFile = LittleFS.open(p, FILE_READ); snapK = k;
            if (!snapFile) { snapK = -1; break; }
        }
        uint32_t rel = off - base, avail = snapSizes[k] - rel;
        if (snapFile.position() != rel) snapFile.seek(rel);
        size_t want = len - got; if (want > avail) want = avail;
        size_t r = want ? snapFile.read(buf + got, want) : 0;
        if (want && r == 0) break;
        got += r; off += r;
        if (off >= base + snapSizes[k]) { base += snapSizes[k]; k++; }
    }
    return got;
}

#if BLE_ENABLED
namespace ble {
uint32_t fileTotal() { return logTotalBytes(); }
uint32_t fileSnapshot() { logSnapshot(); return snapTotal; }
uint32_t fileSnapshotRange(uint32_t after, bool rotate, uint32_t *lo, uint32_t *hi) { logSnapshotRange(after, rotate, lo, hi); return snapTotal; }
size_t   fileRead(uint32_t off, uint8_t *buf, size_t len) { return logReadAt(off, buf, len); }
}
#endif

static void logDump()           // 序列埠倒出全部快取
{
    logSnapshot(); uint32_t total = snapTotal;
    bool lv = liveSerial; liveSerial = false; serialQuiet = true; delay(20);
    Serial.printf("# LOG dump begin bytes=%lu\n", (unsigned long)total);
    static uint8_t buf[1024]; uint32_t off = 0;
    while (off < total) { size_t r = logReadAt(off, buf, sizeof(buf)); if (!r) break; Serial.write(buf, r); off += r; }
    Serial.flush();
    Serial.println("# LOG dump end");
    serialQuiet = false; liveSerial = lv;
}

// ---------- 配戴偵測 ----------
/* 主要看 ACC:連續 WEAR_TIMEOUT_MS (3 分鐘) 加速度完全沒動靜 → 沒配戴。
 * PPG 當補強:同一段時間內若有穩定脈波 (階層式品質 >= WEAR_PULSE_Q,心率 40–180),即使不動 (例如睡著) 仍算配戴。
 * 沒配戴時壓力判定暫停、畫面顯示 NOT WORN、狀態行 worn=0;重新配戴後視窗重新累積。 */

static Biquad   irHpf, irLpf;                            // IR 用同樣的帶通 (與綠光分開的濾波器狀態)
static void wearPushPair(float gFilt, float irRaw)
{
    if (!wearG) return;
    float f = irLpf.process(irHpf.process(irRaw));
    wearG[wearHead] = gFilt; wearI[wearHead] = f;
    wearHead = (wearHead + 1) % WEAR_CORR_N; if (wearFill < WEAR_CORR_N) wearFill++;
}
/* 2026-09-15 抗暫態版:沒戴時拿取、放下、環境光變化會讓綠與 IR 同時出現大暫態,原始相關係數被拉到 ±1
 * (實測放桌上的 10 分鐘裡有 26/114 個視窗 corr ≥ 0.5,綠光 AC 卻只有 1–28 計數)。
 * 改成:兩通道各以「中位數 ± 4 MAD」截幅後再算相關 (暫態不再主導),並要求綠光脈波幅度 (MAD) ≥ WEAR_GMAD_MIN。
 * 同一段資料改算後 0/114 個視窗誤判。wearGMad 給畫面與 weardbg 看。 */
static constexpr float WEAR_GMAD_MIN = 3.0f;      // 綠光帶通後 MAD 下限 (計數):桌上雜訊 0.7–1;戴著實測只有 10.9–17.3 (2026-09-15),拿取暫態截幅後可到 28,所以幅度只擋純雜訊,分辨靠截幅相關 ≥ 0.7 (戴著 0.94–0.97、拿取暫態最高 0.64)
static float wearGMad = 0, wearCorrRaw = 0;
static float *wearTmp = nullptr;                  // 排序暫存 (PSRAM)
static float medianOf(float *v, int n)            // 就地部分排序取中位數 (n=3000 約 1 ms)
{
    std::nth_element(v, v + n / 2, v + n); return v[n / 2];
}
static float *wearGs = nullptr, *wearIs = nullptr;   // 最後 n 筆的副本 (PSRAM)
static float wearCorrCompute(int n = WEAR_CORR_N)
{
    if (wearFill < n) return 0;
    if (!wearTmp) wearTmp = (float *)ps_malloc(sizeof(float) * WEAR_CORR_N);
    if (!wearGs) { wearGs = (float *)ps_malloc(sizeof(float) * WEAR_CORR_N); wearIs = (float *)ps_malloc(sizeof(float) * WEAR_CORR_N); }
    if (!wearGs || !wearIs) return 0;
    for (int k = 0; k < n; k++) { int idx = (wearHead - n + k + WEAR_CORR_N) % WEAR_CORR_N; wearGs[k] = wearG[idx]; wearIs[k] = wearI[idx]; }
    const float *wearG = wearGs, *wearI = wearIs;   // 以下沿用原本的算式,只是資料換成最後 n 筆
    const int WEAR_CORR_N = n;
    // 原始相關 (診斷用)
    double mg = 0, mi = 0;
    for (int k = 0; k < WEAR_CORR_N; k++) { mg += wearG[k]; mi += wearI[k]; }
    mg /= WEAR_CORR_N; mi /= WEAR_CORR_N;
    double sgg = 0, sii = 0, sgi = 0;
    for (int k = 0; k < WEAR_CORR_N; k++) { double a = wearG[k] - mg, b = wearI[k] - mi; sgg += a * a; sii += b * b; sgi += a * b; }
    wearCorrRaw = (sgg > 1e-9 && sii > 1e-9) ? (float)(sgi / sqrt(sgg * sii)) : 0.0f;
    if (!wearTmp) return wearCorrRaw;
    // 截幅:各通道中位數 ± 4 MAD
    memcpy(wearTmp, wearG, sizeof(float) * WEAR_CORR_N); float medG = medianOf(wearTmp, WEAR_CORR_N);
    for (int k = 0; k < WEAR_CORR_N; k++) wearTmp[k] = fabsf(wearG[k] - medG); float madG = medianOf(wearTmp, WEAR_CORR_N) * 1.4826f + 1e-6f;
    memcpy(wearTmp, wearI, sizeof(float) * WEAR_CORR_N); float medI = medianOf(wearTmp, WEAR_CORR_N);
    for (int k = 0; k < WEAR_CORR_N; k++) wearTmp[k] = fabsf(wearI[k] - medI); float madI = medianOf(wearTmp, WEAR_CORR_N) * 1.4826f + 1e-6f;
    wearGMad = madG;
    float loG = medG - 4 * madG, hiG = medG + 4 * madG, loI = medI - 4 * madI, hiI = medI + 4 * madI;
    mg = mi = 0;
    for (int k = 0; k < WEAR_CORR_N; k++) {
        float a = wearG[k] < loG ? loG : (wearG[k] > hiG ? hiG : wearG[k]);
        float b = wearI[k] < loI ? loI : (wearI[k] > hiI ? hiI : wearI[k]);
        wearTmp[k] = a; mg += a; mi += b;
    }
    mg /= WEAR_CORR_N; mi /= WEAR_CORR_N;
    sgg = sii = sgi = 0;
    for (int k = 0; k < WEAR_CORR_N; k++) {
        float b = wearI[k] < loI ? loI : (wearI[k] > hiI ? hiI : wearI[k]);
        double a = wearTmp[k] - mg, bb = b - mi; sgg += a * a; sii += bb * bb; sgi += a * bb;
    }
    float c = (sgg > 1e-9 && sii > 1e-9) ? (float)(sgi / sqrt(sgg * sii)) : 0.0f;
    return madG >= WEAR_GMAD_MIN ? c : 0.0f;       // 幅度不到脈波等級 → 視為沒脈波
}

static void wearUpdate(uint32_t now)
{
    if (irMode == 1) wearCorr = wearCorrCompute();
    else if (irMode == 2) {                      // 間歇:每個 30 s 週期只在相位 29 s 算一次 (IR 已亮 6.5 s),其餘時間沿用上次的值
        static uint32_t lastCycle = 0xFFFFFFFF;
        uint32_t cycle = now / IR_CYCLE_MS, phase = now % IR_CYCLE_MS;
        if (phase >= 29000 && cycle != lastCycle) { lastCycle = cycle; wearCorr = wearCorrCompute(WEAR_DUTY_N); }   // wearUpdate 每 1 s 一次,門檻留 1 s 寬才不會漏掉週期
    } else wearCorr = 0;                          // IR 關:只剩動作判斷
    if (wearCorr >= WEAR_CORR_MIN) lastPulseMs = now;   // 綠光與 IR 看到同一個脈波 → 貼著皮膚
    /* 2026-09-15 規則:最近 30 s 有脈波 → 戴著;沒脈波超過 30 s 時,只有「3 分鐘內有動作且 3 分鐘內有脈波」
     * (例如走動時錶鬆掉) 才繼續算戴著。放桌上 (沒動、沒脈波) 約 45 s 判 OFF;放包包裡晃 (有動、沒脈波) 3 分鐘後 OFF。
     * 壓力判定暫停仍維持 OFF 後 3 分鐘 (WEAR_PAUSE_MS,使用者規格)。 */
    bool pulse30 = now - lastPulseMs < WEAR_NOPULSE_MS;
    /* 2026-09-15 第二輪實測:拿下時的那下動作會讓「3 分鐘內有動作」成立,拿下後 worn 拖了 100 s 以上才 OFF。
     * 改成動作也要是「最近 30 s」的:走動中錶鬆掉 → 動作持續 → 仍算戴著;放桌上 → 動作停 30 s → OFF。 */
    /* 2026-09-18 遲滯:已經是 ON 時,一個 30 s 視窗 corr 偶然掉到 0.7 以下 (手完全靜止時發生過,OFF 1 秒又 ON) 不該就 OFF,
     * 要連續 60 s 沒脈波才切;ON 的條件不變。放桌上判 OFF 約 75 s。 */
    bool w = pulse30 || (worn && now - lastPulseMs < 2 * WEAR_NOPULSE_MS)
             || (wearMotionEver && (now - lastMotionMs < WEAR_NOPULSE_MS) && (now - lastPulseMs < WEAR_TIMEOUT_MS));
    if (wornForce) w = true;
    if (w != worn) {
        statDirty = true;
        worn = w;
        if (!worn) wornOffMs = now;
        Serial.printf("# wear %s (acc std %.3f, g/ir corr %.2f)\n", worn ? "ON" : "OFF", wearAccStd, wearCorr);
        char ln[24]; int n = snprintf(ln, sizeof(ln), "W,%lu,%d\n", (unsigned long)now, worn ? 1 : 0);
        logPush(ln, n);
    }
}

/* 畫面與手機狀態行用的活動名稱:sit / stand 合併成 static (兩者對壓力判定沒有差別,
 * 而且手放桌上時腕角度像站姿,分開顯示只會讓人困惑)。錄音檔 H 行仍寫原始類別。 */
static const char *harLabel()
{
    if (harClass < 0) return "--";
    const char *n = HAR_CLASS_NAMES[harClass];
    if (!strcmp(n, "sit") || !strcmp(n, "stand")) return "static";
    return n;
}

static bool stressPaused = false;  // 壓力判定暫停中 (目前只有未配戴會暫停)

/* 指令處理:序列埠與 BLE 共用。fromBle 時 DUMP 改走 BLE 檔案傳輸。 */
static void handleCommand(const char *cmd, bool fromBle)
{
#if BLE_ENABLED
    if (!strncmp(cmd, "GET,", 4)) {           // 手機拉取檔案的某段
        unsigned long off = 0, len = 0;
        if (sscanf(cmd + 4, "%lu,%lu", &off, &len) == 2) { logFlush(true); ble::sendRange(off, len); }
        return;
    }
    if (!strncmp(cmd, "SYNC,", 5)) {          // 手機增量同步:SYNC,<已收到的最大序號>[,1=先把目前檔收尾]
        unsigned long after = 0; int rot = 0;
        if (sscanf(cmd + 5, "%lu,%d", &after, &rot) >= 1) ble::sendSync((uint32_t)after, rot == 1);
        return;
    }
#endif
    if (strcmp(cmd, "STAT")) screenWake();     // STAT 只是查詢,不喚醒螢幕 (才能觀察休眠狀態)
    if (!strncmp(cmd, "SLEEP,", 6)) { screenSetSleep((uint32_t)atoi(cmd + 6)); return; }
    if (!strncmp(cmd, "LIVE,", 5)) { liveOn = cmd[5] == '1'; if (!liveOn) { liveScore = stressState >= 0 ? stressScoreVal : NAN; liveState = stressState; } Serial.printf("# live score %s\n", liveOn ? "on (every 1 s, screen only)" : "off (screen shows last judgement)"); return; }   // 2026-10-29
    if (!strncmp(cmd, "BATCAL,", 7)) { batCal(atoi(cmd + 7)); return; }
    if (!strncmp(cmd, "ROT,", 4)) { rotSet(atoi(cmd + 4)); return; }
#if CONFIG_PM_ENABLE
    if (!strncmp(cmd, "PM,", 3)) { pmForce(cmd[3] == '0'); return; }
    if (!strncmp(cmd, "NOSLEEP,", 8)) { pmNoSleepSet(cmd[8] == '1'); return; }
    if (!strncmp(cmd, "GPIOSW,", 7)) { esp_sleep_enable_gpio_switch(cmd[7] == '1'); Serial.printf("# gpio sleep switch %s\n", cmd[7] == '1' ? "on (default)" : "off"); return; }
    if (!strcmp(cmd, "PMDUMP")) { esp_pm_dump_locks(stdout); fflush(stdout); return; }
#endif
    if (!strncmp(cmd, "BL,", 3)) {   // 背光診斷:0 = PWM 關;1 = 脫離 PWM 直接拉高 GPIO2;2 = 重新掛 PWM 並設 150
        int m = atoi(cmd + 3);
        if (m == 1) { bool d = ledcDetach(2); pinMode(2, OUTPUT); digitalWrite(2, HIGH); Serial.printf("# BL gpio high (detach=%d)\n", d ? 1 : 0); }
        else if (m == 2) { bool a = ledcAttach(2, 12000, 8); bool w = ledcWrite(2, 150); Serial.printf("# BL pwm attach=%d write=%d duty=%lu freq=%lu\n", a ? 1 : 0, w ? 1 : 0, (unsigned long)ledcRead(2), (unsigned long)ledcReadFreq(2)); }
        else { ledcWrite(2, 0); Serial.println("# BL pwm 0"); }
        return;
    }
    if (!strncmp(cmd, "VERB,", 5)) { liveForce = cmd[5] == '1'; liveSerial = usbPresent || liveForce; Serial.printf("# live serial %s\n", liveSerial ? "on" : "off"); return; }
    if (!strncmp(cmd, "LP,", 3)) { lpSet(cmd[3] == '1'); return; }
    if (!strncmp(cmd, "BG,", 3)) { ble::setBackground(cmd[3] == '1'); return; }   // app 在背景:連線間隔放寬到 200–400 ms
    if (!strncmp(cmd, "SRATE,", 6)) {   // 狀態列心跳秒數 (1 = 每秒,給手錶頁與診斷用;預設 30 = 每次推論一筆)
        int v = atoi(cmd + 6); if (v < 1) v = 1; if (v > 300) v = 300;
        statPeriodMs = (uint32_t)v * 1000UL; statDirty = true;
        Serial.printf("# status every %d s\n", v); return;
    }
    if (!strncmp(cmd, "HUBMS,", 6)) { hubSetBatch((uint32_t)atoi(cmd + 6)); return; }
    if (!strcmp(cmd, "BATLOG")) { batLogDump(); return; }
    if (!strcmp(cmd, "BATCLR")) { LittleFS.remove("/bat.csv"); Serial.println("# bat.csv cleared"); return; }
    if (!strcmp(cmd, "REBOOT")) { Serial.println("# reboot"); delay(50); ESP.restart(); }
    if (!strncmp(cmd, "IR,", 3)) { int v = cmd[3] - '0'; if (v >= 0 && v <= 2) irMode = v; Serial.printf("# IR mode %d (0 off, 1 always, 2 duty)\n", irMode); return; }
    if (!strncmp(cmd, "GYRO,", 5)) { pmBusEnter(); imu.setGyro(cmd[5] == '1'); if (imuFifo) imu.fifoBegin(); pmBusExit(); Serial.printf("# gyro %s\n", cmd[5] == '1' ? "on" : "off"); return; }
    if (!strcmp(cmd, "IMUDBG")) { pmBusEnter(); imu.fifoDebug(); pmBusExit(); return; }
    if (!strncmp(cmd, "IMUFIFO,", 8)) { imuFifoSet(cmd[8] == '1'); return; }
    if (!strncmp(cmd, "PW,", 3)) { int v = atoi(cmd + 3); if (v == 69 || v == 118 || v == 215 || v == 411) hubPwReq = (uint16_t)v; else Serial.println("# PW 69|118|215|411"); return; }
    if (!strncmp(cmd, "HUBRATE,", 8)) { int v = atoi(cmd + 8); if (v == 50 || v == 100 || v == 200 || v == 400) hubRateReq = (uint16_t)v; else Serial.println("# HUBRATE 50|100|200|400"); return; }
    if (!strncmp(cmd, "LOGFMT,", 7)) { int v = atoi(cmd + 7); if (v >= 0 && v <= 2) { logFmt = (uint8_t)v; Serial.printf("# log format %d (0 text, 1 binary, 2 both)\n", v); } return; }
    if (!strncmp(cmd, "TIME,", 5)) { unsigned long ep = 0; int tz = 0; if (sscanf(cmd + 5, "%lu,%d", &ep, &tz) >= 1) timeSet(ep, tz); return; }
    if (!strcmp(cmd, "REC")) logStart();
    else if (!strcmp(cmd, "STOP")) logStop();
    else if (!strcmp(cmd, "DUMP")) {
#if BLE_ENABLED
        if (fromBle) { ble::sendFile(); return; }
#endif
        logDump();
    }
    else if (!strncmp(cmd, "FAST,", 5)) fastSet(cmd[5] == '1');
    else if (!strncmp(cmd, "MASK,", 5)) { maskEnabled = cmd[5] == '1'; Serial.printf("# mask %s\n", maskEnabled ? "on" : "off"); }
    else if (!strncmp(cmd, "MODEL,", 6)) stressModelSet(atoi(cmd + 6));
    else if (!strncmp(cmd, "USB,", 4)) { usbForce = cmd[4] == '0' ? 0 : (cmd[4] == '1' ? 1 : -1); Serial.printf("# usb force %s\n", usbForce < 0 ? "auto" : (usbForce ? "on" : "off (battery, will sleep)")); }
    else if (!strncmp(cmd, "WORN,", 5)) { wornForce = cmd[5] == '1'; Serial.printf("# worn force %s\n", wornForce ? "on (always worn)" : "off (auto)"); }
    else if (!strncmp(cmd, "THR,", 4)) {
        if (!strcmp(cmd + 4, "RESET")) stressThrSet(0, true); else stressThrSet(atof(cmd + 4), false);
    }
    else if (!strcmp(cmd, "STAT")) {
        logFlush(true); uint32_t q[LOG_KEEP + 2]; int n = logChunks(q, LOG_KEEP + 2); uint32_t tot = 0;
        for (int k = 0; k < n; k++) { char p[24]; logChunkName(p, sizeof(p), q[k]); File f = LittleFS.open(p, FILE_READ); uint32_t sz = f ? f.size() : 0; if (f) f.close(); tot += sz; Serial.printf("# c%04lu %lu\n", (unsigned long)q[k], (unsigned long)sz); }
        Serial.printf("# STAT up=%lus rst=%d seq=%lu chunks=%d bytes=%lu fs=%lu/%lu snap=%d %s\n", (unsigned long)(millis() / 1000), (int)esp_reset_reason(),
                      (unsigned long)logSeq, n, (unsigned long)tot, (unsigned long)LittleFS.usedBytes(), (unsigned long)LittleFS.totalBytes(), snapActive ? 1 : 0, batStat());
    }
    else if (!strcmp(cmd, "CLEAR")) { logSnapDrop(); uint32_t q[LOG_KEEP + 2]; int n = logChunks(q, LOG_KEEP + 2); for (int k = 0; k < n; k++) { char p[24]; logChunkName(p, sizeof(p), q[k]); if (q[k] != logSeq) LittleFS.remove(p); } Serial.println("# LOG cleared"); }
    else Serial.printf("# unknown cmd %s\n", cmd);
}

static void logSerialCommands()
{
    static char cmd[32]; static int n = 0;
    if (Serial.available()) lastSerialRxMs = millis();
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            cmd[n] = 0;
            if (n) handleCommand(cmd, false);
            n = 0;
        } else if (n < 31) cmd[n++] = c;
    }
#if BLE_ENABLED
    char bc[32];
    if (ble::takeCommand(bc, sizeof(bc))) { Serial.printf("# BLE cmd %s\n", bc); handleCommand(bc, true); }
#endif
}

static constexpr int BTN_BOOT = 0;   // BOOT 鈕 (GPIO0, 低電位有效):只用來喚醒螢幕

// ---------- 電池:GPIO1 = BAT/3 (板上 200k+100k 分壓);充電偵測靠電壓行為 (板子沒把充電晶片的 CHRG 腳接出來) ----------
static constexpr int PIN_BAT_ADC = 1;
static float    batV = 0;                 // 平滑後電池電壓 (V)
static int      batPct = -1;
static bool     batCharging = false;      // 接著 USB (充電中或已充飽)
static float    batHist[24]; static int batHistN = 0;   // 每 5 秒一筆,共 2 分鐘
static uint32_t batLastMs = 0, batLogMs = 0;
static float    batK = 1.0f;              // 校正係數 (BATCAL,<實測 mV> 由三用電表校正,存 NVS)
// 螢幕方向:ROT,<0-3> 存 NVS (90° 一格);預設 3 = 逆時針轉 90° (2026-09-11 依配戴方向調整)
static int screenRot = 3;
static void rotLoad() { Preferences p; if (p.begin("ui", true)) { screenRot = p.getInt("rot", 3); p.end(); } if (screenRot < 0 || screenRot > 3) screenRot = 3; }
static void rotSet(int r)
{
    if (r < 0 || r > 3) { Serial.println("# ROT: 0-3"); return; }
    screenRot = r; lcd.setRotation(r);
    Preferences p; if (p.begin("ui", false)) { p.putInt("rot", r); p.end(); }
    Serial.printf("# screen rotation %d (x90 deg)\n", r);
}
static void batCalLoad() { Preferences p; if (p.begin("bat", true)) { batK = p.getFloat("k", 1.0f); p.end(); } if (batK < 0.8f || batK > 1.25f) batK = 1.0f; }
static void batCal(int measuredMv)
{
    if (measuredMv < 3000 || measuredMv > 4500 || batV <= 0) { Serial.println("# BATCAL: need 3000-4500 mV and a reading"); return; }
    float rawV = batV / batK;                 // 目前平滑值換回未校正
    batK = measuredMv / 1000.0f / rawV; batV = measuredMv / 1000.0f;
    Preferences p; if (p.begin("bat", false)) { p.putFloat("k", batK); p.end(); }
    Serial.printf("# BATCAL raw=%.3fV measured=%.3fV k=%.4f\n", rawV, measuredMv / 1000.0f, batK);
}
static int batPercent(float v)
{   // 鋰聚合物靜態放電曲線,分段線性
    static const float V[] = {3.30f, 3.50f, 3.60f, 3.70f, 3.80f, 3.90f, 4.00f, 4.10f, 4.20f};
    static const int   P[] = {0,     5,     15,    30,    50,    68,    83,    94,    100};
    if (v <= V[0]) return 0;
    for (int i = 1; i < 9; i++)
        if (v < V[i]) return P[i - 1] + (int)((v - V[i - 1]) / (V[i] - V[i - 1]) * (P[i] - P[i - 1]) + 0.5f);
    return 100;
}
static void batUpdate(uint32_t now)
{
    if (now - batLastMs < 1000) return;
    batLastMs = now;
    // 分壓是 200k+100k,對 ADC 而言源阻抗很高:第一筆丟掉 (讓取樣電容充飽),再平均 16 筆
    analogReadMilliVolts(PIN_BAT_ADC);
    uint32_t mv = 0; for (int i = 0; i < 16; i++) mv += analogReadMilliVolts(PIN_BAT_ADC);
    float v = mv / 16.0f * 3.0f / 1000.0f * batK;
    batV = batV == 0 ? v : batV + 0.2f * (v - batV);
    /* USB 狀態只在「跳變」時切換 (2026-09-11 實測:插上 +145 mV / 拔掉 −40 mV,都在 10 秒內發生);
     * 充飽後充電器停止、負載由電池供應、電壓每分鐘緩降 1–7 mV 直到再充電門檻,這段仍算接著 USB,
     * 所以不能用「電壓下滑」判定拔掉。開機時電壓 ≥ 4.15 V (充電器定電壓段) 視為接著 USB。 */
    static uint32_t tHist = 0; static float v5 = 0;
    if (now - tHist >= 5000) {
        tHist = now;
        if (v5 > 0) { if (batV - v5 >= 0.05f) batCharging = true; else if (v5 - batV >= 0.03f) batCharging = false; }
        else batCharging = batV >= 4.15f;
        v5 = batV;
        if (batHistN < 24) batHist[batHistN++] = batV; else { memmove(batHist, batHist + 1, sizeof(float) * 23); batHist[23] = batV; }
    }
    if (batV >= 4.17f) batCharging = true;      // 只有充電器能把電壓撐到這裡
    /* 2026-09-14 補強:LED 關閉 (未配戴/LP) 時負載很小,拔線壓降不到 30 mV,跳變法抓不到,整段誤判為接著 USB 而不睡。
     * 充飽後充電器停止時電壓仍在 4.08 V 以上 (再充門檻),充電中則持續上升;所以「低於 4.05 V 且 2 分鐘沒上升」只可能是電池供電。 */
    if (batHistN >= 24 && batV < 4.05f && batV <= batHist[0] + 0.003f) batCharging = false;
    /* 2026-09-21 充電中的電量估計:插上 USB 後量到的是充電器電壓 (4.1–4.2 V),照放電曲線會瞬間跳到 99%。
     * 改成:接上 USB 的那一刻記住當時的 % (靜態電壓算的),之後用固定速率往上估 (300 mAh、約 1.2%/分),
     * 電壓 ≥ 4.17 V 持續 20 分鐘 (充電器進入定電壓尾段／已停充) 才顯示 100;拔掉 USB 後 1 分鐘再回到電壓法。 */
    static bool wasChg = false; static uint32_t chgStartMs = 0, cvSinceMs = 0, unplugMs = 0; static int chgStartPct = 0;
    if (batCharging && !wasChg) { chgStartMs = now; chgStartPct = batPct < 0 ? batPercent(batV) : batPct; cvSinceMs = 0; }
    if (!batCharging && wasChg) unplugMs = now;
    wasChg = batCharging;
    if (batCharging) {
        if (batV >= 4.17f) { if (!cvSinceMs) cvSinceMs = now; } else cvSinceMs = 0;
        int est = chgStartPct + (int)((now - chgStartMs) / 60000.0f * 1.2f);
        if (cvSinceMs && now - cvSinceMs >= 20 * 60000UL) est = 100;
        batPct = est > 99 ? (est >= 100 ? 100 : 99) : est;
    } else if (unplugMs && now - unplugMs < 60000) {
        /* 剛拔掉:電壓還在回落,先不更新 (維持上一個值) */
    } else {
        batPct = batPercent(batV);
    }
    if (now - batLogMs >= 10000) {    // 每 10 秒記一筆 (約 3 KB/h),離線可畫充放電曲線
        batLogMs = now; char ln[32]; int n = snprintf(ln, sizeof(ln), "B,%lu,%.3f,%d\n", (unsigned long)now, batV, batCharging ? 1 : 0); logPush(ln, n);
        batFileAppend(now);   // /bat.csv (不輪替,每 60 s 一筆,含耗電歸因欄位;定義在電源管理區塊之後)
    }
}
static void batLogDump()
{
    bool lv = liveSerial; liveSerial = false; serialQuiet = true; delay(20);
    File bf = LittleFS.open("/bat.csv", FILE_READ);
    Serial.printf("# BATLOG %lu bytes\n", bf ? (unsigned long)bf.size() : 0UL);
    if (bf) { uint8_t buf[512]; while (bf.available()) { int k = bf.read(buf, sizeof(buf)); Serial.write(buf, k); } bf.close(); }
    Serial.println("# BATLOG end");
    serialQuiet = false; liveSerial = lv;
}

// ---------- 時間:板子沒有 RTC,手機連上後送 TIME,<unix 秒>,<時區分鐘>,之後用 millis 推算 ----------
static uint32_t timeBaseEpoch = 0, timeBaseMs = 0; static int timeTzMin = 0;
static bool timeNow(struct tm &out)
{
    if (!timeBaseEpoch) return false;
    time_t t = (time_t)timeBaseEpoch + (time_t)((millis() - timeBaseMs) / 1000) + (time_t)timeTzMin * 60;
    gmtime_r(&t, &out); return true;
}
static void timeSet(uint32_t epoch, int tzMin)
{
    timeBaseEpoch = epoch; timeBaseMs = millis(); timeTzMin = tzMin;
    char ln[40]; int n = snprintf(ln, sizeof(ln), "Z,%lu,%lu,%d\n", (unsigned long)timeBaseMs, (unsigned long)epoch, tzMin); logPush(ln, n);   // 離線對應牆鐘
    struct tm tmv; timeNow(tmv);
    Serial.printf("# time set %04d-%02d-%02d %02d:%02d:%02d (tz %+d min)\n", tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday, tmv.tm_hour, tmv.tm_min, tmv.tm_sec, tzMin);
}

// ---------- 螢幕休眠:一段時間沒觸控 / 按鈕 / 指令 / 提示事件就關背光;取樣、判定、BLE 照常 ----------
static uint32_t screenSleepMs = 0;         // 2026-10-29 使用者:「手錶改成持續顯示螢幕」→ 預設不休眠 (10/29 以前是 15000 = 15 秒沒操作關背光);SLEEP,<秒> 可改,0 = 不休眠
static uint32_t lastInteractMs = 0;
static bool     screenOn = true;
static uint32_t screenWakes = 0, screenOnMs = 0, screenOnSince = 0;   // 耗電歸因:螢幕喚醒次數與累計亮著的毫秒
static constexpr uint8_t SCREEN_BRIGHT = 150;
static constexpr int PIN_BL = 2;
static void screenWake()
{
    lastInteractMs = millis();
    if (!screenOn) {
        screenOn = true; screenWakes++; screenOnSince = millis();
#if CONFIG_PM_ENABLE
        pmWakeLock();                                    // 立刻禁止睡眠,別讓面板在重新初始化前後又被放掉
        lcd.getPanel()->setSleep(false); delay(120);
        ledcSetClockSource(LEDC_USE_APB_CLK);            // APB 固定 80 MHz;只在螢幕亮時掛著 (那時本來就不睡)
        ledcAttach(PIN_BL, 12000, 9);                    // 與 LovyanGFX Light_PWM 同參數 (9 bit),之後 setBrightness 的 ledcWrite 才有效
#else
        lcd.getPanel()->setSleep(false); delay(120);
#endif
        lcd.setBrightness(SCREEN_BRIGHT);
        Serial.println("# screen wake");
    }   // 喚醒先看時鐘
}
static void screenUpdate(uint32_t now)
{
    if (screenOn && screenSleepMs && (int32_t)(now - lastInteractMs) >= (int32_t)screenSleepMs) {   // 有號比較:同一圈 loop 內剛喚醒時 now < lastInteractMs
        screenOn = false; lcd.setBrightness(0); lcd.getPanel()->setSleep(true); screenOnMs += now - screenOnSince;
#if CONFIG_PM_ENABLE
        ledcDetach(PIN_BL); pinMode(PIN_BL, OUTPUT); digitalWrite(PIN_BL, LOW);   // 脫離 LEDC:其時脈在 light sleep 會被關掉,醒來 PWM 就死了
#endif
        Serial.println("# screen sleep");
    }
}
/* 省電:
 *  - CPU:螢幕暗且沒在做引導錄製時 80 MHz,其餘 240 MHz (取樣任務用 micros 計時,不受影響)。
 *  - 未配戴 (壓力判定已暫停,即沒動靜也沒脈波超過 6 分鐘) 時把 PPG LED 關掉,只留 IMU 偵測動作;
 *    一動就恢復 (wearUpdate 看到動作 → worn → stressPaused 解除)。
 *    LED 暫存器由取樣任務 (核心 0) 在兩次讀取之間改寫,避免和 hub 的 I2C 讀取撞在一起。 */
#if CONFIG_PM_ENABLE
/* ESP-IDF 電源管理:min 80 MHz (APB 維持 80 MHz,I2C/SPI 時脈不變)、max 240、light sleep。
 * 鎖:pmCpuMax 在螢幕亮或引導錄製時要求 240 MHz;pmNoSleep 在接著 USB (序列埠要能收指令)、BLE 傳檔、
 * 螢幕亮 (SPI DMA 繪圖) 時禁止 light sleep;IMU / 觸控 / hub 讀取的短暫交易也各自持鎖。 */
static esp_pm_lock_handle_t pmCpuMax = nullptr, pmNoSleep = nullptr, pmBus = nullptr;
static bool pmCpuHeld = false, pmSleepHeld = false, pmForceSleep = false, pmNoSleepForce = false;   // pmNoSleepForce:NOSLEEP,1 完全禁止 light sleep (診斷用)   // pmForceSleep:PM,0 指令,插著 USB 也允許 light sleep (測試用)
static void pmHold(esp_pm_lock_handle_t h, bool &held, bool want)
{
    if (!h || held == want) return;
    if (want) esp_pm_lock_acquire(h); else esp_pm_lock_release(h);
    held = want;
}
static void pmBusEnter() { if (pmBus) esp_pm_lock_acquire(pmBus); }
static void pmWakeLock() { pmHold(pmNoSleep, pmSleepHeld, true); pmHold(pmCpuMax, pmCpuHeld, true); }
static void pmNoSleepSet(bool on) { pmNoSleepForce = on; Serial.printf("# light sleep %s\n", on ? "disabled" : "allowed"); }
static void pmForce(bool on) { pmForceSleep = on; Serial.printf("# PM force-sleep %s\n", on ? "on (USB ignored)" : "off"); }
static void pmBusExit()  { if (pmBus) esp_pm_lock_release(pmBus); }
static void pmBegin()
{
    esp_pm_config_t cfg = {}; cfg.max_freq_mhz = 240; cfg.min_freq_mhz = 40; cfg.light_sleep_enable = true;   // 2026-09-18 最低頻 80→40 (晶振頻率)
    esp_err_t e = esp_pm_configure(&cfg);
    // light sleep 期間 GPIO 預設會切到睡眠組態 (輸出腳浮接):面板 RST/CS/DC、背光、hub RESET/MFIO 都會被放掉,
    // 面板因此被重置、喚醒後全黑 (實測連 lcd.init() 都救不回,因為喚醒到取得不睡鎖之間又睡了一次)。
    // 關掉切換,所有腳位在睡眠中維持原狀。
    esp_sleep_enable_gpio_switch(false);
    esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "cpumax", &pmCpuMax);
    esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "nosleep", &pmNoSleep);
    esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "bus", &pmBus);
    Serial.printf("# PM on: light sleep, 40-240 MHz (%s)\n", esp_err_to_name(e));
}
#else
static void pmBusEnter() {}
static void pmBusExit()  {}
static void pmForce(bool) {}
static void pmNoSleepSet(bool) {}
static void pmWakeLock() {}
#endif
static bool lowPower = false, lpForce = false;     // lpForce:LP,1 指令強制進入 (測試用)
static volatile int ledWant = 1, ledApplied = 1;   // 1 = 正常亮度,0 = 關閉
static void powerUpdate(uint32_t now)
{
    /* 2026-09-21 使用者規格:未配戴且加速度 1 分鐘沒動作 → 關 LED;有動作就開燈 (開著就能算綠/IR 相關,戴上即恢復)。
     * 壓力判定的暫停仍是 OFF 滿 3 分鐘 (WEAR_PAUSE_MS),兩者分開。放包包裡晃會亮燈,是規格接受的代價。 */
    bool still = (now - lastMotionMs) >= 60000;
    bool lp = lpForce || (!worn && still);
    if (lp != lowPower) {
        lowPower = lp; ledWant = lp ? 0 : 1;
        Serial.printf("# low power %s\n", lp ? "on (LED off)" : "off (LED on)");
    }
#if CONFIG_PM_ENABLE
    pmHold(pmCpuMax, pmCpuHeld, screenOn);
    bool noSleep = pmNoSleepForce || screenOn || (usbPresent && !pmForceSleep);   // 接著 USB 就不睡 (序列埠指令才收得到)      // 接著 USB 就不睡,序列埠指令才收得到
#if BLE_ENABLED
    noSleep = noSleep || ble::fileBusy;
#endif
    pmHold(pmNoSleep, pmSleepHeld, noSleep);
#else
    uint32_t want = screenOn ? 240 : 80;
    if (getCpuFrequencyMhz() != want) setCpuFrequencyMhz(want);
#endif
}
/* 2026-09-18 另存一份不會被 10 分鐘快取輪替掉的 /bat.csv (重開機也留著),長時間拔線／整顆放電測試用;
 * 每 60 s 一筆 → 256 KB 約 5.7 天;BATLOG 印出、BATCLR 清掉;超過 256 KB 自動清。
 * 2026-09-21 多了歸因欄位 (都是開機後的累計值,分析時取段差):螢幕亮秒數、喚醒次數、BLE 連線次數、傳檔 KB、配戴、LED。
 * 實測這個核心的 FILE_APPEND 每次開檔都覆寫成一筆,所以用 r+ 開檔、seek 到檔尾再寫。 */
static void batFileAppend(uint32_t now)
{
    static uint32_t batFileMs = 0;
    if (now - batFileMs < 60000) return;
    batFileMs = now;
    char ln[112]; int n = snprintf(ln, sizeof(ln), "B,%lu,%.3f,%d,%lu,%lu,%lu,%lu,%d,%d,%lu,%lu\n", (unsigned long)now, batV, batCharging ? 1 : 0,
                 (unsigned long)((screenOnMs + (screenOn ? now - screenOnSince : 0)) / 1000), (unsigned long)screenWakes,
                 (unsigned long)ble::connects, (unsigned long)(ble::txBytes / 1024), worn ? 1 : 0, lowPower ? 0 : 1,
                 (unsigned long)ble::streamPkts, (unsigned long)(ble::fastMs / 1000));   // 第 10–11 欄:串流封包數、快速連線秒數 (儀表板串流／傳檔)
    pmBusEnter();
    File bf = LittleFS.open("/bat.csv", "r+");
    if (!bf) bf = LittleFS.open("/bat.csv", "w");
    if (bf && bf.size() > 262144) { bf.close(); LittleFS.remove("/bat.csv"); bf = LittleFS.open("/bat.csv", "w"); }
    if (bf) { bf.seek(bf.size()); bf.write((const uint8_t *)ln, n); bf.close(); }
    pmBusExit();
}
static void ledApply()          // 取樣任務內呼叫
{
    int key = ledWant * 2 + irWant;
    if (ledApplied == key) return;
    ledApplied = key;
    /* 2026-09-21 修:這幾筆 hub I2C 寫入原本沒拿 pmBus 鎖,電池模式下晶片可能在交易中間進 light sleep,寫入就丟掉;
     * 另外 40 MHz 時 hub 暫存器讀回會亂 (0xFE/隨機),所以寫的時候同時拉到 240 MHz。讀回不可靠,不做讀回驗證。
     * hub raw 第 1 組資料 (韌體叫 green,壓力鏈用) 其實是 IR LED (0x0D);第 2 組 (叫 ir) 才是綠光 LED (0x0E)。 */
    pmBusEnter();
#if CONFIG_PM_ENABLE
    if (pmCpuMax) esp_pm_lock_acquire(pmCpuMax);
#endif
    bioHub.writeRegisterMAX30101(REG_LED2_PA, ledWant ? IR_BRIGHTNESS : 0x00);                    // IR = 壓力鏈通道,常亮
    delay(2);
    bioHub.writeRegisterMAX30101(REG_LED3_PA, (ledWant && irWant) ? GREEN_BRIGHTNESS : 0x00);   // 綠光 = 參考通道
    delay(2);
#if CONFIG_PM_ENABLE
    if (pmCpuMax) esp_pm_lock_release(pmCpuMax);
#endif
    pmBusExit();
    Serial.printf("# LED regs written: ir(chain)=%s green(ref)=%s\n", ledWant ? "on" : "off", (ledWant && irWant) ? "on" : "off");
}
static void lpSet(bool on) { lpForce = on; }
static void screenSetSleep(uint32_t sec) { screenSleepMs = sec * 1000UL; screenWake(); Serial.printf("# screen sleep %lus\n", (unsigned long)sec); }
static const char *batStat()
{
    static char b[96];
    snprintf(b, sizeof(b), "bat=%.3fV (k=%.3f) %d%% chg=%d usb=%d screen=%d cpu=%lu lp=%d fmt=%d bl=%lu/%luHz", batV, batK, batPct, batCharging ? 1 : 0, usbPresent ? 1 : 0, screenOn ? 1 : 0, (unsigned long)getCpuFrequencyMhz(), lowPower ? 1 : 0, (int)logFmt, (unsigned long)ledcRead(2), (unsigned long)ledcReadFreq(2));
    return b;
}

static float processSample(uint32_t rawGreen, uint32_t nowMs)
{
    float x = (float)rawGreen;
#if BLE_ENABLED
    ble::pushRaw(rawGreen, lastIr, nowMs);   // 原始 100 Hz 綠光 + IR 給手機存檔
#endif

    /* --- 接觸偵測已完全移除 ----------------------------------------------
     * 原本 fingerOn 會閘住濾波、峰值偵測與壓力視窗,並在轉態時清空緩衝;
     * 之後降級成畫面上的指示燈;現在連指示燈也拿掉了,一律處理。
     *
     * 代價要講清楚:手錶沒戴在身上時,環境光雜訊一樣會被算成 HRV 並產生
     * CALM/STRESS 判定,那個結果沒有意義。畫面上的 RR 數字是唯一的判斷依據
     * (30 秒視窗正常有 35-45 個)。
     */
    if (warmup < WARMUP_N) { // 開機後先讓讀值穩定
        warmup++;
        dcBase = (warmup == 1) ? x : fminf(dcBase, x);
        if (warmup == WARMUP_N) {   // 暖機結束:預載濾波器,只做這一次
            hpf.primeDC(x); lpf.reset();
            resetDetectors(nowMs); d4Reset();
            lastBeatMs = nowMs;
            blankUntil = nowMs + BLANK_MS;
        }
        return 0;
    }

    float y = lpf.process(hpf.process(x));
    if (nowMs < blankUntil) return y;   // 開機空白期是 DC 階躍暫態,不可餵進分析

    runDetectors(y, nowMs);
#if STRESS_ENABLED
    stressFeed(x);      // 壓力辨識吃的是原始讀值,自己有一條 Butterworth
#endif
    return y;
}

static void pushSample(float v)
{
    waveBuf[waveHead] = v;
    waveHead = (waveHead + 1) % WAVE_LEN;
}

/* 電池圖示在畫面頂端 (圓形螢幕 y=4 處可見寬度約 ±30px),百分比小字在底端 */
static void drawBattery()
{
    const int x = 90, y = 7, w = 22, h = 10;    // 圖示 + 百分比一起置中在頂端 (y=7 處可見寬度約 ±40px)
    uint16_t col = batCharging ? COL_WAVE : batPct > 50 ? COL_WAVE : batPct > 20 ? 0xFD20 : COL_HR;
    frame.drawRect(x, y, w, h, COL_DIM); frame.fillRect(x + w, y + 3, 2, 4, COL_DIM);
    int fw = batPct < 0 ? 0 : (w - 4) * batPct / 100;
    if (fw > 0) frame.fillRect(x + 2, y + 2, fw, h - 4, col);
    if (batCharging) {   // 閃電
        int cx = x + w / 2;
        frame.fillTriangle(cx + 2, y + 1, cx - 3, y + 6, cx + 1, y + 6, TFT_WHITE);
        frame.fillTriangle(cx - 2, y + 9, cx + 3, y + 4, cx - 1, y + 4, TFT_WHITE);
    }
    char b[8]; snprintf(b, sizeof(b), "%d%%", batPct < 0 ? 0 : batPct);
    frame.setTextDatum(top_left); frame.setFont(&fonts::Font2); frame.setTextColor(COL_DIM, COL_BG);
    frame.drawString(b, x + w + 6, y - 3);
}

/* 2026-09-21 藍牙連線圖示 (電池圖示左邊):連上手機 = 藍色實線,沒連 (廣播中) = 灰色細線。三頁都畫 (跟電池一起)。 */
static void drawLink()
{
    const int cx = 76, y = 6, h = 12;
    uint16_t col = ble::connected ? 0x3D9F : COL_DIM;
    int t = y, b = y + h, ru = y + 3, rl = b - 3;
    frame.drawLine(cx, t, cx, b, col);
    frame.drawLine(cx, t, cx + 4, ru, col); frame.drawLine(cx + 4, ru, cx - 4, rl, col);
    frame.drawLine(cx, b, cx + 4, rl, col); frame.drawLine(cx + 4, rl, cx - 4, ru, col);
    if (ble::connected) { frame.drawLine(cx + 1, t, cx + 1, b, col); }   // 連線時加粗一點
}

/* 時鐘頁 (2026-10-29 改版,使用者:「持續顯示螢幕,每秒更新一次壓力狀態」):上半時間與日期;中間即時壓力狀態
 * (每秒的即時分數 vs 判定線;LIVE,0 時是最近一次正式判定);下半心率 (「上次正式判定幾秒前」那一行 10/29 當天拿掉:判定已改每秒)。
 * 顏色:紅 = 壓力、綠 = 平穩、灰 = 沒資料／未配戴。時間由手機同步,沒同步前顯示 --:--。
 * 10/29 以前:時間 (y 96)、日期 (138)、BPM (178;紅 = 正式判定為壓力)。 */
static void drawClockPage()
{
    struct tm t; bool ok = timeNow(t); char b[40];
    frame.setTextDatum(middle_center);
    frame.setFont(&fonts::Font7);                       // 7 段大字
    frame.setTextColor(ok ? TFT_WHITE : COL_DIM, COL_BG);
    if (ok) snprintf(b, sizeof(b), "%02d:%02d", t.tm_hour, t.tm_min); else snprintf(b, sizeof(b), "--:--");
    frame.drawString(b, 120, 52);
    frame.setFont(&fonts::Font2); frame.setTextColor(COL_DIM, COL_BG);
    if (ok) {
        static const char *WD[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
        snprintf(b, sizeof(b), "%04d-%02d-%02d %s", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, WD[t.tm_wday]);
    } else snprintf(b, sizeof(b), "open app to sync time");
    frame.drawString(b, 120, 86);
    // 中間:即時壓力狀態
    const char *label; uint16_t col;
    if (stressPaused)         { label = "OFF WRIST"; col = COL_DIM; }
    else if (liveState == 1)  { label = "STRESS";    col = COL_HR; }
    else if (liveState == 0)  { label = "CALM";      col = COL_WAVE; }
    else if (liveState == -1) { label = "NO SIGNAL"; col = COL_DIM; }    // 算過但訊號平坦／RR 不足
    else                      { label = "WAIT";      col = COL_DIM; }    // 視窗還沒滿 120 秒
    frame.setFont(&fonts::Font4); frame.setTextColor(col, COL_BG);
    frame.drawString(label, 120, 118);
    frame.setFont(&fonts::Font2); frame.setTextColor(col, COL_BG);
    if (!stressPaused && !isnan(liveScore)) snprintf(b, sizeof(b), "%.2f / thr %.2f", (double)liveScore, (double)stressThr);
    else if (!stressPaused && liveState == -2 && stressFill < STRESS_WIN) snprintf(b, sizeof(b), "filling %ds", (STRESS_WIN - stressFill) / (int)PPG_FS);
    else snprintf(b, sizeof(b), "-- / thr %.2f", (double)stressThr);
    frame.drawString(b, 120, 142);
    // 下半:心率 (顏色跟正式判定,同 10/29 以前) 與上次正式判定幾秒前
    uint16_t hc = (stressState == 1 && !stressPaused) ? COL_HR : COL_WAVE;
    frame.setFont(&fonts::Font4); frame.setTextColor(hc, COL_BG);
    snprintf(b, sizeof(b), "%u", (unsigned)hierBpm); frame.drawString(b, 120, 176);
    frame.setFont(&fonts::Font2); frame.setTextColor(hc, COL_BG); frame.drawString("bpm", 120, 202);   // 2026-10-29 使用者:「把下面的幾秒鐘前判斷拿掉」→ 判定已是每秒一次,那一行沒意義
}

// (drawWavePage / drawMainPage 於 2026-09-22 移除:螢幕只留時鐘頁)

#if SELF_TEST
// 取 N 筆平均與標準差,用來判斷讀值是否「活的」
static void sampleStats(float &mean, float &sd)
{
    const int N = 40;
    float v[N];
    for (int i = 0; i < N; i++) { v[i] = (float)bioHub.readSensor().irLed; delay(12); }
    mean = 0; for (int i = 0; i < N; i++) mean += v[i]; mean /= N;
    sd = 0; for (int i = 0; i < N; i++) sd += (v[i] - mean) * (v[i] - mean);
    sd = sqrtf(sd / N);
}

// 測一組設定:LED 全關 vs 全開,讀值有明顯差異才代表資料路徑是活的
static void probe(const char *name, uint8_t ledReg)
{
    float offMean, offSd, onMean, onSd, dump;
    // 沉降要夠久:切換 LED 後訊號有暫態,量太早會得到反向的假結果
    bioHub.writeRegisterMAX30101(ledReg, 0x00);
    delay(1500); sampleStats(dump, dump); sampleStats(offMean, offSd);
    bioHub.writeRegisterMAX30101(ledReg, 0xFF);
    delay(1500); sampleStats(dump, dump); sampleStats(onMean, onSd);
    float delta = onMean - offMean;
    Serial.printf("# %-22s off=%.0f(sd%.0f) on=%.0f(sd%.0f) delta=%.0f  %s\n",
                  name, offMean, offSd, onMean, onSd, delta,
                  (fabsf(delta) > 200 || onSd > 50) ? "<== LIVE" : "frozen");
}

/**
 * 量測 hub 真正的資料更新率。
 * 以固定週期輪詢並記錄「值有變化」的次數 — 重複值代表 hub 還沒產生新資料。
 * 量測期間完全不印字,避免序列埠阻塞干擾結果 (結果先存陣列,事後才輸出)。
 */
static void measureFresh(uint16_t hubRate, uint32_t pollMs)
{
    bioHub.begin(Wire1, PIN_PPG_RST, PIN_PPG_MFIO);
    bioHub.configSensor();
    bioHub.setPulseWidth(411);
    bioHub.setSampleRate(hubRate);
    delay(1000);
    bioHub.agcAlgoControl(0); delay(100);
    bioHub.writeRegisterMAX30101(REG_MODE_CONFIG, 0x07);
    bioHub.writeRegisterMAX30101(REG_SLOT_1_2, SLOT_GREEN_ONLY);
    bioHub.writeRegisterMAX30101(REG_SLOT_3_4, 0x00);
    bioHub.writeRegisterMAX30101(REG_LED3_PA, GREEN_BRIGHTNESS);
    bioHub.writeRegisterMAX30101(REG_LED1_PA, 0x00);
    bioHub.writeRegisterMAX30101(REG_LED2_PA, 0x00);
    delay(300);

    const int N = 300;
    static uint32_t v[N];
    uint32_t t0 = millis();
    for (int i = 0; i < N; i++) {
        uint32_t next = t0 + (i + 1) * pollMs;
        v[i] = bioHub.readSensor().irLed;
        while ((int32_t)(millis() - next) < 0) { /* 精準等到下一個週期 */ }
    }
    uint32_t elapsed = millis() - t0;

    int fresh = 0, maxRun = 0, run = 1, longStalls = 0;
    for (int i = 1; i < N; i++) {
        if (v[i] != v[i - 1]) {
            fresh++;
            if (run > maxRun) maxRun = run;
            if (run >= 5) longStalls++;
            run = 1;
        } else run++;
    }
    Serial.printf("# hubRate=%u poll=%lums -> fresh=%.1fHz (%d/%d new) maxStall=%d longStalls=%d\n",
                  hubRate, (unsigned long)pollMs,
                  fresh * 1000.0f / elapsed, fresh, N - 1, maxRun, longStalls);
}

static void runRateTest()
{
    Serial.println("# ===== sensor rate test =====");
    measureFresh(100, 10);  // 目前設定
    measureFresh(50, 10);
    measureFresh(200, 10);
    measureFresh(400, 10);
    measureFresh(100, 20);  // 放慢輪詢
    Serial.println("# ===== rate test done =====");
}

// 逐步加上我改過的設定,找出是哪一項讓資料凍結
static void runConfigBisect()
{
    Serial.println("# ===== config bisect =====");

    // A: 完全不寫任何原始暫存器,用 hub 預設 (紅光+IR)
    bioHub.begin(Wire1, PIN_PPG_RST, PIN_PPG_MFIO);
    bioHub.configSensor(); bioHub.setPulseWidth(411); bioHub.setSampleRate(100);
    delay(1000);
    probe("A hub default (IR)", REG_LED2_PA);

    // B: multi-LED + 綠光/紅光雙 slot (之前看到會變動的設定)
    bioHub.begin(Wire1, PIN_PPG_RST, PIN_PPG_MFIO);
    bioHub.configSensor(); bioHub.setPulseWidth(411); bioHub.setSampleRate(100);
    delay(1000);
    bioHub.agcAlgoControl(0); delay(100);
    bioHub.writeRegisterMAX30101(REG_MODE_CONFIG, 0x07);
    bioHub.writeRegisterMAX30101(REG_SLOT_1_2, 0x13);
    bioHub.writeRegisterMAX30101(REG_SLOT_3_4, 0x00);
    delay(100);
    probe("B multiLED green+red", REG_LED3_PA);

    // C: B + 只留綠光 slot
    bioHub.writeRegisterMAX30101(REG_SLOT_1_2, 0x03); delay(100);
    probe("C green slot only", REG_LED3_PA);

    // D: C + 4x 硬體平均
    bioHub.writeRegisterMAX30101(REG_FIFO_CONFIG, FIFO_CFG_AVG4); delay(100);
    probe("D + FIFO avg4", REG_LED3_PA);

    // E: D + 內部 400Hz / ADC 範圍調整
    bioHub.writeRegisterMAX30101(REG_SPO2_CONFIG, SPO2_CFG_WRIST); delay(100);
    probe("E + SPO2 400Hz", REG_LED3_PA);

    Serial.println("# ===== bisect done =====");
}
#endif

static volatile uint32_t drawUs = 0;  // 繪圖耗時 (診斷用)
static volatile uint32_t hierUs = 0;  // 階層式運算耗時 (診斷用)
static volatile uint32_t acUs = 0;    // 自相關運算耗時 (診斷用)

// 把一筆 (可能是內插出來的) 樣本送進處理鏈
static void emitSample(float green, uint32_t nowMs, bool interpolated = false)
{
    float filt = processSample(green, nowMs);
    if (nowMs >= blankUntil) d4Feed(green, nowMs);

    if (nowMs >= blankUntil) wearPushPair(filt, (float)lastIr);   // 配戴偵測:綠 / IR 相關
    portENTER_CRITICAL(&dataMux);
    pushSample(filt);
    // 空白期內是貼上造成的 DC 階躍暫態,不可餵進自相關視窗,否則剛貼上的
    // 幾秒內讀值會被這段暫態主導而亂跳
    if (nowMs >= blankUntil) { acPush(filt); hierPush(filt); }
    portEXIT_CRITICAL(&dataMux);

    // 用韌體自己的時間戳,離線分析才能精確重建處理鏈看到的訊號
    // (主機端時間戳因序列埠成批到達而不可靠)。內插樣本也一併輸出並標記。
    if (liveSerial) Serial.printf("%lu,%.0f,%.1f,%u,%d,%.2f,%u,%u,%u,%d\n",
                  (unsigned long)nowMs, green, filt, hierBpm, 1,
                  hierQuality, acBpm, bpm, d4Bpm, interpolated ? 1 : 0);
    if (logRec) {
        if (logFmt != 1) { char ln[40]; int n = snprintf(ln, sizeof(ln), "P,%lu,%.0f,%lu\n", (unsigned long)nowMs, green, (unsigned long)lastIr); logPush(ln, n); }
        if (logFmt != 0) logPpgBin(nowMs, (uint32_t)lrintf(green), lastIr);   // lrintf 與 %.0f 同為四捨六入五成雙,內插樣本兩邊一致
    }
}

/**
 * PPG 取樣任務 — 固定跑在核心 0
 *
 * 繪圖一次要 ~23ms (240x240 sprite),若和取樣擠在同一核心會把取樣率壓到 50Hz,
 * 而濾波器係數是照 100Hz 設計的,取樣率不對會讓通帶整個偏掉 (0.7Hz 高通在
 * 50Hz 下實際變成 1.4Hz,直接吃掉心跳訊號)。因此取樣獨立一顆核心、用
 * 微秒級計時維持精準週期,繪圖再慢也不影響。
 */
/* hub 批次讀取 (2026-09-11 省電):MAX32664 每個指令要等 2 ms,逐筆讀 (每 10 ms 一次) 讓 I2C 佔掉 24% 的時間、
 * 也讓 light sleep 進不去。改成每 HUB_BATCH_MS 讀一次輸出 FIFO 的樣本數,再一次把所有樣本讀回 (raw 模式每筆 12 bytes),
 * 放進佇列;之後仍以 10 ms 步進跑原本的每筆處理:佇列空時沿用上一筆 (等同以前讀到重複值),停頓內插邏輯完全不變。 */
/* 2026-09-11 再省電:批次週期 50 → 200 ms (每次 20 筆),核心 0 醒來次數少四倍。hub 輸出 FIFO 夠深 (數十筆以上),
 * Wire 緩衝加大到 512 bytes 才能一次讀 20–40 筆 (initSensor 內 setBufferSize)。週期可用 HUBMS,<ms> 指令即時改 (50–500,測試用,不存 NVS)。 */
static uint32_t hubBatchMs   = 500;                                    // 每次批次讀取的週期 (2026-09-18 200→500:核心 0 醒來次數 5→2 次/秒)
static constexpr int      HUB_SAMPLE_B   = 12;                         // raw 模式每筆 4 LED × 3 bytes
static constexpr int      HUB_MAX_READ   = 84;                         // 84 × 12 + 1 狀態 = 1009 bytes ≤ Wire 緩衝 1024 (500 ms = 50 筆)
static constexpr uint8_t  HUB_ADDR       = 0x55;
static constexpr int      HUB_Q          = 64;
static uint32_t hubQ[HUB_Q][2]; static int hubQHead = 0, hubQCount = 0;
static void hubSetBatch(uint32_t ms)
{
    if (ms < 50) ms = 50;
    if (ms > 500) ms = 500;
    hubBatchMs = ms - ms % SAMPLE_MS;
    Serial.printf("# hub batch %lu ms (%lu samples)\n", (unsigned long)hubBatchMs, (unsigned long)(hubBatchMs / SAMPLE_MS));
}
static uint32_t hubDropped = 0, hubReadErr = 0;
static int hubReadBatch()          // 回傳讀到幾筆
{
    int n = bioHub.numSamplesOutFifo();                 // 指令 0x12/0x00 (含 2 ms 等待)
    if (n <= 0 || n == 0xFF) return 0;
    if (n > HUB_MAX_READ) n = HUB_MAX_READ;
    Wire1.beginTransmission(HUB_ADDR); Wire1.write(0x12); Wire1.write(0x01);   // READ_DATA_OUTPUT / READ_DATA
    if (Wire1.endTransmission() != 0) { hubReadErr++; return 0; }
    delay(2);
    int want = n * HUB_SAMPLE_B + 1;
    int got = Wire1.requestFrom((int)HUB_ADDR, want);
    if (got != want) { hubReadErr++; while (Wire1.available()) Wire1.read(); return 0; }
    uint8_t st = Wire1.read();
    if (st != 0) { hubReadErr++; while (Wire1.available()) Wire1.read(); return 0; }
    static uint8_t buf[HUB_MAX_READ * HUB_SAMPLE_B];
    for (int i = 0; i < n * HUB_SAMPLE_B; i++) buf[i] = Wire1.read();
    for (int i = 0; i < n; i++) {
        const uint8_t *p = buf + i * HUB_SAMPLE_B;
        uint32_t g = ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2];   // 第 1 組:實際是 IR LED (0x0D) —— 壓力鏈通道 (變數名沿用 g/green,歷史因素)
        uint32_t ir = ((uint32_t)p[3] << 16) | ((uint32_t)p[4] << 8) | p[5];  // 第 2 組:實際是綠光 LED (0x0E) —— 參考通道 (間歇,只給配戴偵測)
        if (hubQCount == HUB_Q) { hubQHead = (hubQHead + 1) % HUB_Q; hubQCount--; hubDropped++; }
        int t = (hubQHead + hubQCount) % HUB_Q; hubQ[t][0] = g; hubQ[t][1] = ir; hubQCount++;
    }
    hubGot += n;
    return n;
}

static void ppgTask(void *)
{
    uint32_t nextUs = micros();
    uint32_t tDiag = millis(), sampleCount = 0, readUs = 0;
    uint32_t lastGreen = 0, staleCount = 0;
    int pendingDupes = 0;

    for (;;) {
        /* 每 hubBatchMs (預設 200 ms) 醒一次:先批次讀 hub,再以 10 ms 步進處理 HUB_TICKS 個 tick (時間戳往回推,代表過去一段)。
         * 用 vTaskDelay 讓出到剩不到 1 ms 再自旋補零頭,其餘時間核心 0 可以 light sleep。 */
        const int HUB_TICKS = (int)(hubBatchMs / SAMPLE_MS);
        nextUs += hubBatchMs * 1000;
        int32_t remainUs = (int32_t)(nextUs - micros());
        if (remainUs > 1500) vTaskDelay(pdMS_TO_TICKS((remainUs - 1000) / 1000));
        while ((int32_t)(nextUs - micros()) > 0) { /* 自旋 <1ms */ }
        if ((int32_t)(micros() - nextUs) > 500000) nextUs = micros();   // 落後太多 (例如 light sleep 喚醒延遲累積) 就重新對齊
        uint32_t nowBatch = millis();

        if (!sensorOk) { // 初始化失敗 → 每 3 秒重試
            static uint32_t tRetry = 0;
            if (nowBatch - tRetry >= 3000) { tRetry = nowBatch; sensorOk = initSensor(); }
            continue;
        }

        if (hubPwReq) {
            // setPulseWidth 會讓 hub 重設感測器 (LED 電流回到 hub 預設、AGC 可能重開):實測手腕 DC 64k→26k、脈波消失。
            // 所以改完要關 AGC 並強制 ledApply 重寫 LED 電流。
            pmBusEnter(); bioHub.setPulseWidth(hubPwReq); bioHub.agcAlgoControl(0); pmBusExit();
            ledApplied = -1;
            Serial.printf("# hub pulse width -> %u us (LED re-applied)\n", hubPwReq); hubPwReq = 0;
        }
        if (hubRateReq) {   // 取樣率改在這裡套用,避免和批次讀取搶 I2C
            pmBusEnter(); bioHub.setSampleRate(hubRateReq); pmBusExit();
            Serial.printf("# hub sample rate -> %u Hz\n", hubRateReq); hubRateReq = 0;
        }
        irWant = irMode == 1 || (irMode == 2 && (millis() % IR_CYCLE_MS) >= IR_ON_FROM_MS);
        ledApply();
        uint32_t t0 = micros();
        pmBusEnter();
        hubReadBatch();
        pmBusExit();
        readUs = micros() - t0;

        for (int tick = 0; tick < HUB_TICKS; tick++) {
            uint32_t now = nowBatch - (uint32_t)(HUB_TICKS - 1 - tick) * SAMPLE_MS;
            uint32_t green;
            if (hubQCount > 0) { green = hubQ[hubQHead][0]; lastIr = hubQ[hubQHead][1]; hubQHead = (hubQHead + 1) % HUB_Q; hubQCount--; }
            else green = lastGreen;          // 佇列空 = 以前的「重複讀值」,走同一條停頓內插
            sampleCount++;

            /* --- 停頓內插 ---------------------------------------------------
             * hub 會週期性卡住十幾個樣本。若直接把重複值送進濾波器,訊號會變成「平台 + 突跳」的階梯波,
             * 階梯邊緣是寬頻的、又以固定週期重複,正好落在通帶內、被自相關鎖成假心率。
             * 因此偵測到重複值時先不處理,等新值到達再把中間用線性內插補齊,送進處理鏈的仍是均勻的 100 Hz。 */
            if (green == lastGreen) {
                staleCount++;
                if (++pendingDupes < 60) continue; // 尚未有新資料,先等
                // 超過 0.6 秒沒新資料:視為感測器停擺,不再等待
            }

            int gaps = pendingDupes;
            pendingDupes = 0;
            for (int k = 1; k <= gaps; k++) {   // 這些樣本代表「過去」那幾個時刻,時間戳往回推
                float v = lastGreen + (float)((double)green - (double)lastGreen) * k / (gaps + 1);
                emitSample(v, now - (uint32_t)(gaps - k + 1) * SAMPLE_MS, true);
            }
            emitSample((float)green, now);
            lastGreen = green;
        }

        // 每 5 秒回報實際取樣率 (以 # 開頭方便過濾)
        if (nowBatch - tDiag >= 5000) {
            acUpdates = 0;
            if (!serialQuiet) Serial.printf("# acUs=%lu hierUs=%lu fires A=%lu C=%lu D=%lu votes=%lu\n",
                          (unsigned long)acUs, (unsigned long)hierUs,
                          (unsigned long)fireCount[0], (unsigned long)fireCount[1],
                          (unsigned long)fireCount[2], (unsigned long)voteCount);
            if (!serialQuiet) Serial.printf("# weardbg corr=%.2f raw=%.2f gmad=%.1f nRR=%d accstd=%.3f worn=%d sinceMotion=%lus sincePulse=%lus\n", wearCorr, wearCorrRaw, wearGMad, wearLastNRR, wearAccStd, worn ? 1 : 0, (unsigned long)((millis() - lastMotionMs) / 1000), (unsigned long)((millis() - lastPulseMs) / 1000));
            fireCount[0] = fireCount[1] = fireCount[2] = 0; voteCount = 0;
            if (!serialQuiet) Serial.printf("# fs_actual=%.1fHz stale=%.0f%% hub=%.1fHz read=%luus draw=%luus hubq=%d drop=%lu err=%lu batch=%lums\n",
                          sampleCount * 1000.0f / (nowBatch - tDiag),
                          sampleCount ? staleCount * 100.0f / sampleCount : 0,
                          hubGot * 1000.0f / (nowBatch - tDiag),
                          (unsigned long)readUs, (unsigned long)drawUs, hubQCount, (unsigned long)hubDropped, (unsigned long)hubReadErr, (unsigned long)hubBatchMs);
            tDiag = nowBatch;
            sampleCount = 0;
            staleCount = 0;
            hubGot = 0;
        }
    }
}

static void drawErrorPage(const char *msg)
{
    frame.fillScreen(COL_BG);
    frame.setTextDatum(middle_center);
    frame.setFont(&fonts::Font4);
    frame.setTextColor(COL_HR, COL_BG);
    frame.drawString("Sensor Error", 120, 100);
    frame.setFont(&fonts::Font2);
    frame.setTextColor(COL_DIM, COL_BG);
    frame.drawString(msg, 120, 140);
    frame.pushSprite(0, 0);
}

void setup()
{
    Serial.begin(921600); // 提高鮑率:100Hz 全量輸出才不會阻塞取樣任務
    gpio_pulldown_en((gpio_num_t)PIN_UART_RX); gpio_pullup_dis((gpio_num_t)PIN_UART_RX);   // USB 偵測用 (見 usbPresent)
    delay(200);
#if STRESS_ENABLED
    stressBuf  = (float *)ps_malloc(sizeof(float) * STRESS_WIN);
    stressWork = (float *)ps_malloc(sizeof(float) * STRESS_WIN);
    { float e = 0; uint32_t t = micros(); bool okc = cnnSelfTest(&e); Serial.printf("# cnn selftest %s err=%.2e us=%lu\n", okc ? "ok" : "FAIL", e, (unsigned long)(micros() - t)); }
    accRing = (float (*)[3])ps_malloc(sizeof(float) * 3 * ACC_RING);
    acc64   = (float (*)[3])ps_malloc(sizeof(float) * 3 * STRESS_WIN);
    mExt = (float *)ps_malloc(sizeof(float) * MASK_TMP); mOut = (float *)ps_malloc(sizeof(float) * MASK_TMP); mNorm = (float *)ps_malloc(sizeof(float) * MASK_TMP);
    if (!stressBuf || !stressWork || !accRing || !acc64 || !mExt || !mOut || !mNorm) Serial.println("# PSRAM alloc FAILED");
#endif
#if STRESS_ENABLED
    stressSelfTest();   // 開機自檢:整條處理鏈 vs PC 端
    maskSelfTest();     // 遮罩自檢 vs tools/stft_mask_ref.py
#endif

    // 帶通 0.7–3.5 Hz @ fs=100Hz
    hpf.highpass(FS, HPF_FC);
    lpf.lowpass(FS, LPF_FC);
    irHpf.highpass(FS, HPF_FC); irLpf.lowpass(FS, LPF_FC);
    wearG = (float *)ps_malloc(sizeof(float) * WEAR_CORR_N); wearI = (float *)ps_malloc(sizeof(float) * WEAR_CORR_N);
    accelLpf.lowpass(FS, 5.0f); // 二階微分會放大高頻,需再低通

    pinMode(BTN_BOOT, INPUT_PULLUP);
    pinMode(PIN_BAT_ADC, INPUT); analogSetPinAttenuation(PIN_BAT_ADC, ADC_11db); batCalLoad();
    lastInteractMs = millis();
    lastMotionMs = lastPulseMs = millis();   // 開機先視為配戴 30 s (等 corr 緩衝填滿);沒動作也沒脈波就 OFF (wearMotionEver 擋掉開機那筆假動作)
#if STRESS_ENABLED
    stressThrLoad();
#endif
#if BLE_ENABLED
    ble::begin("PPG Watch");
#endif
    logFsOk = LittleFS.begin(true);   // 掛載失敗就格式化 (spiffs 分割區 3.4 MB)
    Serial.printf("# LittleFS %s total=%lu used=%lu\n", logFsOk ? "ok" : "FAIL",
                  (unsigned long)(logFsOk ? LittleFS.totalBytes() : 0),
                  (unsigned long)(logFsOk ? LittleFS.usedBytes() : 0));
    logBegin();

    lcd.init();
    rotLoad(); lcd.setRotation(screenRot);
    lcd.setBrightness(SCREEN_BRIGHT);
#if CONFIG_PM_ENABLE
    pmBegin();
#endif
#if HAR_ENABLED
    imu.begin();        // 必須在 lcd.init() 之後:I2C0 由 LovyanGFX 初始化,IMU 共用它
    if (imu.ok) imuFifoSet(true);   // 2026-09-18 預設 FIFO 批次讀
    harSelfTest();
#endif

    /* 關閉震動馬達。
     * 板上兩個 MOSFET 開關接點在 GPIO4 (MOSFET1) 與 GPIO5 (MOSFET2),震動馬達
     * 一般焊在這裡。程式若不管它,閘極會浮接而可能讓馬達半導通、持續震動,
     * 所以明確拉低。
     * GPIO5 同時是觸控中斷 TP_INT (LovyanGFX 需要它當輸入),不能拉低,
     * 若你的馬達接在 MOSFET2 請改焊到 MOSFET1。
     */
    // 畫布 115 KB:PM 版 (Arduino 3 / IDF 5.5 / NimBLE 2) 內部 RAM 最大連續區塊只剩 ~34 KB,放內部會失敗 → 全黑。
    // 先放 PSRAM (10 fps 下每幀多十幾 ms 可接受),PSRAM 不可用才退回內部 RAM。
    frame.setColorDepth(16);
    bool spriteOk = false;
    if (ESP.getFreePsram() > 200000) { frame.setPsram(true); spriteOk = frame.createSprite(240, 240); }
    if (!spriteOk) { frame.setPsram(false); spriteOk = frame.createSprite(240, 240); }
    if (!spriteOk)
        Serial.printf("# createSprite FAILED (free heap %u, largest block %u, psram %u)\n",
                      (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap(), (unsigned)ESP.getFreePsram());
    else
        Serial.printf("# sprite ok (%s), free heap %u, largest block %u, psram free %u\n", frame.getBuffer() && esp_ptr_external_ram(frame.getBuffer()) ? "PSRAM" : "internal",
                      (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap(), (unsigned)ESP.getFreePsram());

    drawErrorPage("Init MAX32664...");
    sensorOk = initSensor();

#if SELF_TEST
    if (sensorOk) {
        Serial.println("# --- which LED is actually active? ---");
        probe("GREEN (LED3)", REG_LED3_PA);
        bioHub.writeRegisterMAX30101(REG_LED3_PA, GREEN_BRIGHTNESS);
        delay(300);
        probe("RED   (LED1)", REG_LED1_PA);
        bioHub.writeRegisterMAX30101(REG_LED1_PA, 0x00);
        delay(300);
        probe("IR    (LED2)", REG_LED2_PA);
        bioHub.writeRegisterMAX30101(REG_LED2_PA, 0x00);
        bioHub.writeRegisterMAX30101(REG_LED3_PA, GREEN_BRIGHTNESS);
        Serial.println("# --- done ---");
    }
#endif
    if (!sensorOk) {
        Serial.println("MAX32664/MAX30101 init failed! Check wiring.");
        drawErrorPage("Check wiring: SDA15 SCL16");
    }
    Serial.println("ms,green,filt,bpm,contact,quality,acbpm,peakbpm,d4bpm,interp");

    // 取樣獨立跑在核心 0,與核心 1 的繪圖/觸控互不干擾
    xTaskCreatePinnedToCore(ppgTask, "ppg", 4096, nullptr, 3, nullptr, 0);
}

void loop()
{
    static uint32_t tDraw = 0, tAutoCorr = 0, tHier = 0, tStress = 0, tHar = 0;
    uint32_t now = millis();

    if (!sensorOk) { // 感測器未就緒:取樣任務會自動重試,這裡只顯示提示
        drawErrorPage("Check wiring: SDA15 SCL16");
        delay(200);
        return;
    }

    // 自相關與階層式都每 1 秒更新 (跑在核心 1,不影響取樣時序)
    if (now - tAutoCorr >= 1000) {
        tAutoCorr = now;
        uint32_t t0 = micros();
        acUpdate();
        acUs = micros() - t0;
    }
    if (now - tHier >= 1000) {
        tHier = now;
        uint32_t t0 = micros();
        hierUpdate();
        hierUs = micros() - t0;
    }

    logSerialCommands();
    logFlush();
    {   // BOOT 鈕 (GPIO0, 低電位有效):喚醒螢幕
        static uint32_t lastBtn = 0;
        if (digitalRead(BTN_BOOT) == LOW && now - lastBtn > 2000) { lastBtn = now; screenWake(); }
    }
    if (fastUntil && (int32_t)(now - fastUntil) >= 0) fastSet(false);   // 校正模式逾時保護
    {   // USB 偵測 (每秒):RX 線高 = CH343 有電 = USB 接著
        static uint32_t tUsb = 0; static bool verbose = false; (void)verbose;
        if (now - tUsb >= 1000) { tUsb = now; usbPresent = usbForce >= 0 ? (usbForce == 1) : ((now - lastSerialRxMs < 120000) || batCharging); liveSerial = usbPresent || liveForce; }
    }
    batUpdate(now);
    screenUpdate(now);
    powerUpdate(now);
#if BLE_ENABLED
    ble::tick(now);
#endif
#if BLE_ENABLED
    {   /* 推一行狀態給手機。2026-09-23:改成「事件驅動 + 心跳」——有新推論或 worn/充電/REC 改變就立刻送,
         * 否則每 statPeriodMs (預設 30 s,等於一次推論) 送一筆當心跳讓 app 更新電量。
         * 校正期間自動每秒一筆 (STRESS_STEP_CALIB);SRATE,<秒> 可讓 app 在需要時要求加快。 */
        static uint32_t tBle = 0;
        const uint32_t period = (calibMode || STRESS_STEP <= STRESS_STEP_CALIB) ? 1000u : statPeriodMs;   // 2026-10-29 正式判定每秒 → 每秒一筆
        if (ble::connected && (statDirty || now - tBle >= period)) {
            tBle = now; statDirty = false;
            char ln[112];
            // S,ms,score,thr,state,nRR,har,rec,fast,hr,worn,gIRcorr,vbat,chg,alt,pct (app/lib/ble.dart 必須一致;alt = 另一個模型的分數,NaN 表示無;pct = 電量 %)
            char altS[16]; if (isnan(stressAltVal)) strcpy(altS, "NaN"); else snprintf(altS, sizeof(altS), "%.2f", stressAltVal);
            snprintf(ln, sizeof(ln), "S,%lu,%.2f,%.2f,%d,%d,%s,%d,%d,%u,%d,%.2f,%.2f,%d,%s,%d",
                     (unsigned long)now, stressScoreVal, (double)stressThr, stressState, stressNRR,
                     harLabel(), logMarked ? 1 : 0, calibMode ? 1 : 0, (unsigned)hierBpm,   // fast 欄 = 校正模式 (2026-10-29 起不再等於「每秒判定」)
                     stressPaused ? 0 : 1, wearCorr, batV, batCharging ? 1 : 0, altS, batPct);   // 第 16 欄 = 手錶算的電量 % (充電中用時間估,不會瞬間 99%)   // worn 欄 = 是否仍在判定 (未配戴超過 3 分鐘才為 0)
            ble::sendStatus(ln);
        }
    }
#endif

#if HAR_ENABLED
    /* IMU 20 Hz 取樣放在核心 1:Bus0 與 LovyanGFX 的觸控驅動共用同一個 I2C 周邊,
     * 從核心 0 讀會逾時 (Error 263) 並把 PPG 任務卡到 10 Hz。
     * 繪圖最多 ~13ms,對 50ms 週期的抖動可接受 (特徵是 2 秒視窗的 mean/std)。 */
    {
        static uint32_t imuNextUs = 0;
        uint32_t us = micros();
        const uint32_t period = imuFifo ? 200000 : 50000;
        if ((int32_t)(us - imuNextUs) >= 0) {
            imuNextUs = (imuNextUs == 0) ? us + period : imuNextUs + period;
            if ((int32_t)(us - imuNextUs) > 2 * (int32_t)period) imuNextUs = us + period;  // 落後太多就重新對齊
            if (imuFifo) harFifoPoll(); else harSample();
        }
    }
    if (now - tHar >= 1000) { tHar = now; harUpdate(); }
#endif

#if STRESS_ENABLED
    // 壓力辨識:每秒檢查一次,實際是否推論由 stressCount 控制
    if (now - tStress >= 1000) {
        tStress = now;
        wearUpdate(now);
        bool mv = !worn && (now - wornOffMs >= WEAR_PAUSE_MS);   // 未配戴「持續超過 3 分鐘」才暫停;短暫誤判不會清掉視窗
        if (mv != stressPaused) {
            Serial.printf("# stress %s (worn=%d)\n", mv ? "paused" : "resumed", worn ? 1 : 0);
            if (!mv) stressReset();         // 重新戴上:丟掉舊資料,從乾淨資料重新累積
            stressPaused = mv;
        }
        if (!mv) { uint32_t n0 = stressJudgeN; stressUpdate(); stressLiveUpdate(stressJudgeN != n0); }   // 2026-10-29 正式判定 (30 s 一次) 之後,每秒一次即時分數給螢幕
        else { liveScore = NAN; liveState = -2; }
    }
#endif

    static uint32_t tTouch = 0;
    if (now - tTouch >= (screenOn ? 50u : 200u)) {   // 觸控 20 Hz;螢幕暗時 5 Hz (少醒幾次,喚醒延遲最多 200 ms)
        tTouch = now;
        static bool wasTouched = false;
        int32_t tx, ty;
        static uint32_t lastTouchMs = 0;
        pmBusEnter(); bool raw = lcd.getTouch(&tx, &ty); pmBusExit();
        if (raw) lastTouchMs = now;
        bool touched = now - lastTouchMs < 150;     // 去彈跳:20 Hz 輪詢時 CST816S 會交替回報有/無
        if (touched && !wasTouched) {
            screenWake();                       // 只有一頁 (時鐘):觸控只負責喚醒
        }
        wasTouched = touched;
    }

    if (screenOn && now - tDraw >= 100) {      // 10 fps (時鐘頁其實 1 fps 就夠,保留給觸控回饋)
        tDraw = now;
        uint32_t t0 = micros();
        frame.fillScreen(COL_BG);
        drawClockPage(); drawBattery(); drawLink();
        frame.pushSprite(0, 0);
        drawUs = micros() - t0;
    }
    /* 2026-09-18:螢幕暗著時主迴圈最短排程是 IMU (FIFO 200 ms / 輪詢 50 ms) 與觸控 200 ms,不必每 5 ms 醒;
     * 原本 vTaskDelay(5) 讓整顆晶片每 5 ms 醒一次,是 light sleep 最大的浪費。螢幕亮著仍 5 ms (觸控 20 Hz、10 fps)。 */
    vTaskDelay(screenOn ? 5 : (imuFifo ? 100 : 50));
}
