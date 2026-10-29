/**
 * 活動辨識隨機森林推論 —— 與 tools/train_har.py 的 forest_predict_py() 相同邏輯:
 *   截斷 → 標準化 → 每棵樹從根走到葉 (子節點索引是該樹內的相對索引,要加樹的偏移)
 *   → 葉機率累加 → argmax。與 sklearn.predict 完全相同 (它平均的是機率)。
 */
#pragma once
#include <math.h>
#include "har_forest.h"

static inline int harForestInfer(const float *x, float *prob)
{
    float z[HARF_N_FEAT];
    for (int i = 0; i < HARF_N_FEAT; i++) {
        float v = x[i];
        if (v < HARF_CLIP_LO[i]) v = HARF_CLIP_LO[i];
        if (v > HARF_CLIP_HI[i]) v = HARF_CLIP_HI[i];
        float sc = (fabsf(HARF_SCALE[i]) > 1e-12f) ? HARF_SCALE[i] : 1.0f;
        z[i] = (v - HARF_MEAN[i]) / sc;
    }
    for (int c = 0; c < HARF_N_CLASS; c++) prob[c] = 0;
    for (int t = 0; t < HARF_N_TREE; t++) {
        int base = HARF_TREE_OFF[t], i = base;
        while (HARF_FEAT[i] >= 0)
            i = base + ((z[HARF_FEAT[i]] <= HARF_THR[i]) ? HARF_LEFT[i] : HARF_RIGHT[i]);
        for (int c = 0; c < HARF_N_CLASS; c++) prob[c] += HARF_LEAF[i][c];
    }
    int best = 0;
    for (int c = 1; c < HARF_N_CLASS; c++) if (prob[c] > prob[best]) best = c;
    return best;
}
