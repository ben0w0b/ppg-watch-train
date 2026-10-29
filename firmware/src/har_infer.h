/**
 * 活動辨識推論 (LDA 多類) —— 與 PC 端相同:截斷 → 標準化 → 各類線性分數 → argmax
 */
#pragma once
#include <math.h>
#include "har_model.h"

static inline int harInfer(const float *x, float *scores)
{
    float z[HAR_N_FEAT];
    for (int i = 0; i < HAR_N_FEAT; i++) {
        float v = x[i];
        if (v < HAR_CLIP_LO[i]) v = HAR_CLIP_LO[i];
        if (v > HAR_CLIP_HI[i]) v = HAR_CLIP_HI[i];
        float sc = (fabsf(HAR_SCALE[i]) > 1e-12f) ? HAR_SCALE[i] : 1.0f;
        z[i] = (v - HAR_MEAN[i]) / sc;
    }
    int best = 0;
    for (int c = 0; c < HAR_N_CLASS; c++) {
        float s = HAR_B[c];
        for (int i = 0; i < HAR_N_FEAT; i++) s += HAR_W[c][i] * z[i];
        scores[c] = s;
        if (s > scores[best]) best = c;
    }
    return best;
}
