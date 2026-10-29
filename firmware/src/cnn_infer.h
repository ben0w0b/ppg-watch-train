/**
 * tiny 1D-CNN 壓力推論(不逐拍)—— 與 tools/export_tiny_cnn.py 的 ref_forward 同一算法。
 *
 *   輸入:stressBuf 的 120 s × 64 Hz 視窗(0.5–10 Hz 帶通 + MA3,係數由 src/ppg_sos.h 產生;時間順序)
 *   → 每 CNN_DEC 取 1(16 Hz,CNN_NIN 樣本)→ z-score(母體 std)
 *   → 三層 Conv1d(k=7, same padding)-ReLU-MaxPool(4, floor) → 全域平均 → Linear → logit
 *
 * 算量約 0.97 M 乘加,ESP32-S3 浮點約 0.1–0.2 s;暫存放 PSRAM(最大一層 8×1920 float = 60 KB)。
 * logit > 門檻 判為壓力;門檻由 app 校正(與 LR 分數同一套流程)。
 */
#pragma once

#include <math.h>
#include "cnn_model.h"

static float *cnnBufA = nullptr, *cnnBufB = nullptr;   // ping-pong 暫存 (PSRAM)
static float *cnnIn = nullptr;                          // 輸入 (PSRAM,避免佔內部 RAM)

static bool cnnInit()
{
    if (cnnBufA) return true;
    int maxc = 0; for (int i = 1; i <= CNN_NLAYER; i++) if (CNN_CH[i] > maxc) maxc = CNN_CH[i];
    size_t n = (size_t)maxc * CNN_NIN;
    cnnBufA = (float *)ps_malloc(sizeof(float) * n);
    cnnBufB = (float *)ps_malloc(sizeof(float) * n);
    cnnIn   = (float *)ps_malloc(sizeof(float) * CNN_NIN);
    return cnnBufA && cnnBufB && cnnIn;
}

/* 一層:in[ci][n] → out[co][n/4]。W 以 [co][ci][K] 排列。 */
static int cnnLayer(const float *in, int ci, int n, const float *W, const float *b, int co, float *out)
{
    const int P = CNN_K / 2;
    const int m = n / 4;
    for (int o = 0; o < co; o++) {
        const float *wo = W + (size_t)o * ci * CNN_K;
        float *dst = out + (size_t)o * m;
        for (int j = 0; j < m; j++) {
            float best = -1e30f;
            for (int q = 0; q < 4; q++) {
                int t = j * 4 + q;                     // 輸出位置 (same padding)
                float acc = b[o];
                for (int c = 0; c < ci; c++) {
                    const float *src = in + (size_t)c * n;
                    const float *wk = wo + c * CNN_K;
                    int lo = t - P;
                    if (lo >= 0 && lo + CNN_K <= n) {  // 不碰邊界:全展開
                        acc += wk[0] * src[lo] + wk[1] * src[lo + 1] + wk[2] * src[lo + 2] + wk[3] * src[lo + 3]
                             + wk[4] * src[lo + 4] + wk[5] * src[lo + 5] + wk[6] * src[lo + 6];
                    } else {
                        for (int k = 0; k < CNN_K; k++) { int idx = lo + k; if (idx >= 0 && idx < n) acc += wk[k] * src[idx]; }
                    }
                }
                if (acc < 0) acc = 0;                  // ReLU
                if (acc > best) best = acc;            // MaxPool4
            }
            dst[j] = best;
        }
    }
    return m;
}

/* w64: 長度 n64 (= CNN_NIN*CNN_DEC) 的 64 Hz 視窗,時間順序。回傳 logit;失敗 (std≈0) 回 false。 */
static bool cnnInfer(const float *w64, int n64, float *outLogit)
{
    if (!cnnInit() || n64 < CNN_NIN * CNN_DEC) return false;
    double mean = 0; for (int i = 0; i < CNN_NIN; i++) { cnnIn[i] = w64[i * CNN_DEC]; mean += cnnIn[i]; }
    mean /= CNN_NIN;
    double var = 0; for (int i = 0; i < CNN_NIN; i++) { double d = cnnIn[i] - mean; var += d * d; }
    float sd = (float)sqrt(var / CNN_NIN);
    if (sd < 1e-9f) return false;
    for (int i = 0; i < CNN_NIN; i++) cnnIn[i] = (cnnIn[i] - (float)mean) / sd;

    const float *W[CNN_NLAYER] = {(const float *)CNN_W0, (const float *)CNN_W1, (const float *)CNN_W2};
    const float *B[CNN_NLAYER] = {CNN_B0, CNN_B1, CNN_B2};
    const float *cur = cnnIn; float *a = cnnBufA, *bb = cnnBufB; int n = CNN_NIN;
    for (int l = 0; l < CNN_NLAYER; l++) {
        n = cnnLayer(cur, CNN_CH[l], n, W[l], B[l], CNN_CH[l + 1], a);
        cur = a; float *t = a; a = bb; bb = t;
    }
    int co = CNN_CH[CNN_NLAYER];
    float s = CNN_BL;
    for (int o = 0; o < co; o++) {
        const float *ch = cur + (size_t)o * n; float m = 0;
        for (int j = 0; j < n; j++) m += ch[j];
        s += CNN_WL[o] * (m / n);
    }
    *outLogit = s;
    return true;
}

/* 開機自檢:用匯出的測試向量比對 PC 端 logit。 */
static bool cnnSelfTest(float *err)
{
    float lg;
    if (!cnnInfer(CNN_TEST_X, CNN_NIN * CNN_DEC, &lg)) return false;
    *err = fabsf(lg - CNN_TEST_LOGIT);
    return *err < 1e-3f;
}
