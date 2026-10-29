/**
 * scipy.signal.find_peaks 的 C++ 移植 —— 手錶韌體與離線比對共用同一份實作。
 *
 * 與 scipy 相同的三個階段與相同順序:
 *   1. 局部極大 (平台取中點)
 *   2. distance 篩選 (依峰高由高到低,刪掉鄰近範圍內較低的峰)
 *   3. prominence 篩選 (拓樸突出度:往兩側走到遇見更高的樣本為止,取較高的那個谷底)
 */
#pragma once

#include <math.h>
#include <string.h>

static constexpr float PROM_SIGMA = 2.2f;
static constexpr int PEAK_DISTANCE = 0; // 峰間最小距離 (樣本數),0 = 不限制
static constexpr int MAX_CAND = 2000;       // 候選峰上限 (120 s × 3.5 Hz = 420 個振盪峰 + 漣漪)
// 4 秒子視窗只需數十個,但壓力辨識的 60 秒視窗有 6000 點,帶通後最多可能出現
// 3.5Hz x 60s = 210 個振盪峰,加上雜訊漣漪。220 會讓 findPeaks 掃到一半就停止
// (迴圈條件含 nc < MAX_CAND),等於默默截掉後半段視窗。

static int findPeaks(const float *x, int n, float minProm, int distance,
                     int *outIdx, float *outAmp, int maxOut)
{
    static int cand[MAX_CAND];
    static bool keep[MAX_CAND];
    int nc = 0;

    // --- 1. 局部極大 (平台取中點) ---
    int i = 1;
    while (i < n - 1 && nc < MAX_CAND) {
        if (x[i - 1] < x[i]) {
            int iAhead = i + 1;
            while (iAhead < n - 1 && x[iAhead] == x[i]) iAhead++;
            if (x[iAhead] < x[i]) {
                cand[nc++] = (i + iAhead - 1) / 2; // 平台中點
                i = iAhead;
            }
        }
        i++;
    }
    if (nc == 0) return 0;
    for (int k = 0; k < nc; k++) keep[k] = true;

    // --- 2. 距離篩選:按峰高由高到低走訪,保留高的、刪掉其鄰近範圍內的 ---
    // distance <= 1 等同不篩選,直接跳過 (省掉 O(n^2) 的排序)
    if (distance > 1) {
    static int order[MAX_CAND];
    for (int k = 0; k < nc; k++) order[k] = k;
    for (int a = 0; a < nc; a++)                    // 依峰高遞減排序
        for (int b = a + 1; b < nc; b++)
            if (x[cand[order[b]]] > x[cand[order[a]]]) {
                int t = order[a]; order[a] = order[b]; order[b] = t;
            }
    for (int oi = 0; oi < nc; oi++) {
        int k = order[oi];
        if (!keep[k]) continue;                     // 已被更高的峰刷掉
        for (int m = k - 1; m >= 0 && cand[k] - cand[m] < distance; m--) keep[m] = false;
        for (int m = k + 1; m < nc && cand[m] - cand[k] < distance; m++) keep[m] = false;
    }
    }

    // --- 3. 突出度篩選 ---
    int np = 0;
    for (int k = 0; k < nc && np < maxOut; k++) {
        if (!keep[k]) continue;
        int pk = cand[k];
        float leftMin = x[pk], rightMin = x[pk];
        for (int j = pk; j >= 0 && x[j] <= x[pk]; j--) if (x[j] < leftMin) leftMin = x[j];
        for (int j = pk; j < n && x[j] <= x[pk]; j++) if (x[j] < rightMin) rightMin = x[j];
        float prom = x[pk] - fmaxf(leftMin, rightMin);
        if (prom >= minProm) { outIdx[np] = pk; outAmp[np] = x[pk]; np++; }
    }
    return np;
}
