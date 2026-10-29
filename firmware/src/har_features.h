/**
 * 活動辨識特徵 —— 與 tools/train_har.py 的 window_features() 逐行對應
 *
 *   2 秒視窗 (40 筆 @ 20 Hz) 的加速度 (m/s²) 與角速度 (rad/s):
 *   各軸 mean/std、合成量 mean/std/min/max (加速度) 共 10 維;陀螺儀 8 維已停用 (HAR_N_FEAT_ > 10 才編譯)。
 *   全部是純算術,板上可逐位重現。np.std 為 ddof=0。
 */
#pragma once
#include <math.h>
#include <string.h>

#define HAR_FS      20
#define HAR_WIN_SEC 2
#define HAR_WIN     (HAR_FS * HAR_WIN_SEC)   // 40
#define HAR_N_FEAT_ 10   // 2026-09-18 陀螺儀關掉:只算加速度 10 維 (train_har.py --acc-only)

static inline void harMeanStd(const float *v, int n, float *mean, float *sd)
{
    double s = 0, s2 = 0;
    for (int i = 0; i < n; i++) { s += v[i]; s2 += (double)v[i] * v[i]; }
    double m = s / n, var = s2 / n - m * m;
    *mean = (float)m; *sd = (float)sqrt(var > 0 ? var : 0);
}

/* acc, gyr: (n × 3)。out[18] 順序必須與 har_model.h 的 HAR_FEAT_NAMES 相同。 */
static void harFeatures(const float (*acc)[3], const float (*gyr)[3], int n, float *out)
{
    float col[HAR_WIN], mag[HAR_WIN];
    float m, sd;
    // 加速度各軸 mean (0-2) / std (3-5)
    for (int c = 0; c < 3; c++) {
        for (int i = 0; i < n; i++) col[i] = acc[i][c];
        harMeanStd(col, n, &m, &sd); out[c] = m; out[3 + c] = sd;
    }
    // 加速度合成量 mean/std/min/max (6-9)
    float mn = 1e30f, mx = -1e30f;
    for (int i = 0; i < n; i++) {
        mag[i] = sqrtf(acc[i][0]*acc[i][0] + acc[i][1]*acc[i][1] + acc[i][2]*acc[i][2]);
        if (mag[i] < mn) mn = mag[i];
        if (mag[i] > mx) mx = mag[i];
    }
    harMeanStd(mag, n, &m, &sd); out[6] = m; out[7] = sd; out[8] = mn; out[9] = mx;
#if HAR_N_FEAT_ > 10
    // 陀螺儀各軸 mean (10-12) / std (13-15)
    for (int c = 0; c < 3; c++) {
        for (int i = 0; i < n; i++) col[i] = gyr[i][c];
        harMeanStd(col, n, &m, &sd); out[10 + c] = m; out[13 + c] = sd;
    }
    // 陀螺儀合成量 mean/std (16-17)
    for (int i = 0; i < n; i++)
        mag[i] = sqrtf(gyr[i][0]*gyr[i][0] + gyr[i][1]*gyr[i][1] + gyr[i][2]*gyr[i][2]);
    harMeanStd(mag, n, &m, &sd); out[16] = m; out[17] = sd;
#else
    (void)gyr;
#endif
}
