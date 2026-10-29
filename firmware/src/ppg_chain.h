/**
 * 手錶端的 PPG 處理鏈 —— 與 tools/extract_wesad.py 逐行對應
 *
 *     100 Hz 原始訊號
 *       → 重取樣到 64 Hz          (WESAD 的 E4 手腕 BVP 是 64 Hz;所有以樣本數
 *                                  表示的常數 —— MA3 的跨度、300ms 不應期、
 *                                  RR 量化階距 —— 都必須在同一個取樣率下才一致)
 *       → Butterworth 0.5-10 Hz order 2 (兩級串接 biquad,係數取自 scipy)
 *       → MA3 置中平滑            (串流版,延遲 1 個樣本)
 *       → 60 秒視窗 / 步長 10 秒
 *       → MA3 置中平滑            (每個視窗再做一次,邊緣以端點延拓)
 *       → z-score → find_peaks(prominence = 0.3, distance = 400ms;見 hrv_features.h)
 *
 * PC 端還有一步「std 標準化」沒有移植:帶通的直流增益為零,而每個視窗後面還會
 * 做 z-score,所以全域的平移與縮放在穩態下會被完全抵消,不影響峰值位置。
 */
#pragma once

#include <math.h>

// ---------- 取樣率 ----------
static constexpr float PPG_FS_IN  = 100.0f;   // 感測器
static constexpr float PPG_FS     = 64.0f;    // 處理鏈 (與 WESAD 相同)
// 用整數時基避免長時間累積漂移:1/1600 秒為單位,
// 輸入每筆前進 16、輸出每筆前進 25 (16*100 = 25*64 = 1600)
static constexpr uint32_t PPG_TICK_IN  = 16;
static constexpr uint32_t PPG_TICK_OUT = 25;

// ---------- Butterworth 帶通 ----------
// 係數由 tools/extract_wesad.py 自動產生到 ppg_sos.h,與 PC 端保證同一組。
#include "ppg_sos.h"

/* scipy sosfilt 的每級運算 (direct form II transposed,與 lfilter 相同):
 *     y = b0*x + s1
 *     s1 = b1*x - a1*y + s2
 *     s2 = b2*x - a2*y                                                   */
struct PpgSos {
    float s1[PPG_NSOS], s2[PPG_NSOS];
    void reset() { for (int i = 0; i < PPG_NSOS; i++) s1[i] = s2[i] = 0; }
    float process(float x)
    {
        for (int i = 0; i < PPG_NSOS; i++) {
            const float *c = PPG_SOS[i];
            float y = c[0] * x + s1[i];
            s1[i] = c[1] * x - c[3] * y + s2[i];
            s2[i] = c[2] * x - c[4] * y;
            x = y;
        }
        return x;
    }
};


/**
 * 串流處理鏈:餵 100 Hz 樣本,吐出 64 Hz、已帶通並平滑的樣本。
 * MA3 置中需要往後看一個樣本,所以輸出比輸入延遲一拍。
 */
struct PpgChain {
    PpgSos   sos;
    uint32_t tIn, tOut;      // 整數時基
    float    prevIn;         // 前一筆 100Hz 樣本 (線性內插用)
    bool     haveIn;
    float    m0, m1;         // MA3 的滑動視窗 (m0 最舊)
    int      mFill;

    void reset()
    {
        sos.reset();
        tIn = tOut = 0; prevIn = 0; haveIn = false;
        m0 = m1 = 0; mFill = 0;
    }

    /* 餵一筆 100Hz 樣本。每產生一筆 64Hz 輸出就呼叫 sink(v)。 */
    template <typename F>
    void push(float x, F sink)
    {
        if (!haveIn) { prevIn = x; haveIn = true; tIn = PPG_TICK_IN; return; }

        // --- 重取樣:輸出時刻落在 (tIn - TICK_IN, tIn] 之間就內插一筆 ---
        while (tOut <= tIn) {
            float frac = (float)(tOut - (tIn - PPG_TICK_IN)) / (float)PPG_TICK_IN;
            float r = prevIn + frac * (x - prevIn);
            tOut += PPG_TICK_OUT;

            // --- 帶通 ---
            float b = sos.process(r);

            // --- MA3 置中 (延遲一拍) ---
            if (mFill < 2) { if (mFill == 0) m0 = b; else m1 = b; mFill++; }
            else {
                sink((m0 + m1 + b) * (1.0f / 3.0f));
                m0 = m1; m1 = b;
            }
        }
        prevIn = x;
        tIn += PPG_TICK_IN;
    }
};

/**
 * 視窗內的置中 MA3,邊緣以端點值延拓 —— 對應 Python 的
 *     x = concat(sig[0]*ones(3), sig, sig[-1]*ones(3))
 *     convolve(w, x, 'same')[3:-3]
 * 效果就是 out[i] = mean(in[i-1], in[i], in[i+1]),越界處取端點值。
 * (原始碼的 fwd/bwd 平均在對稱核下與此等價,連邊緣也一樣。)
 */
static void ppgSmoothInPlace(float *v, int n)
{
    if (n <= 1) return;
    float pm = v[0];                       // 保留被覆寫前的 v[i-1]
    v[0] = (pm + pm + v[1]) * (1.0f / 3.0f);
    for (int i = 1; i < n - 1; i++) {
        float cur = v[i];
        v[i] = (pm + cur + v[i + 1]) * (1.0f / 3.0f);
        pm = cur;
    }
    v[n - 1] = (pm + v[n - 1] + v[n - 1]) * (1.0f / 3.0f);
}
