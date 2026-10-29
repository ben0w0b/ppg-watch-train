/**
 * WESAD 壓力二分類的 LDA 推論 —— 手錶韌體與兩支測試韌體共用同一份實作。
 *
 *     x = clip(x, CLIP_LO, CLIP_HI)     與 PC 端同一組 winsorize 邊界
 *     z = (x - MEAN) / SCALE
 *     score = dot(W, z) + B             score > 0 判為壓力
 *
 * 沒有近似、沒有量化,與 PC 端是同一個算式,差異只來自 float64 vs float32。
 */
#pragma once

#include <math.h>
#include "stress_model.h"

// 標準差為零的特徵沒有資訊,標準化會變成 0/0。這類特徵在 PC 端 (prepare()) 就會
// 先被移除,所以正常不會出現;這道防呆是為了萬一有漏網的常數特徵。
// 取 1.0f 與 sklearn StandardScaler 的行為相同 (_handle_zeros_in_scale),
// 因此加上它是讓兩端更一致,而不是引入差異。
static constexpr float STRESS_SCALE_EPS = 1e-12f;

static inline float stressInfer(const float *x)
{
    float s = STRESS_B;
    for (int i = 0; i < STRESS_N_FEAT; i++) {
        float v = x[i];
        if (v < STRESS_CLIP_LO[i]) v = STRESS_CLIP_LO[i];
        if (v > STRESS_CLIP_HI[i]) v = STRESS_CLIP_HI[i];
        float sc = (fabsf(STRESS_SCALE[i]) > STRESS_SCALE_EPS) ? STRESS_SCALE[i] : 1.0f;
        s += STRESS_W[i] * ((v - STRESS_MEAN[i]) / sc);
    }
    return s;
}
