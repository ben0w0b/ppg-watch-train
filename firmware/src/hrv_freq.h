/**
 * HRV 頻域特徵 —— 與 tools/extract_wesad.py 的 freq_features() 逐行對應
 *
 *     RR (ms) → 累積時間 → 線性內插到 256 點均勻網格 → 去均值
 *     → 256 點 FFT → |X/N|² → 頻帶內功率相加
 *
 * 原專案用 scipy UnivariateSpline(k=3) 的平滑樣條內插到 1 kHz 再做 6 萬點 FFT。
 * 那條路徑無法在 MCU 上重現 (FITPACK 的節點選擇與平滑度求解是數百行 Fortran),
 * 而且是訓練資料裡出現 1e48 發散值的來源。改用線性內插 + 固定點數 FFT,
 * 兩端都能精確執行。
 *
 * 注意 60 秒視窗的頻率解析度只有 1/60 ≈ 0.0167 Hz:VLF 頻帶 (0.003-0.04) 只落得到
 * 約 2 個頻率格,ULF (<0.003) 完全無法解析 —— 因此不計算 ULF。
 */
#pragma once

#include <math.h>
#include <string.h>

#define HRV_N_FREQ 7
#define FD_NFFT    256      // 固定點數,兩端才會得到同一組頻率格
#define FD_MIN_RR  8        // 少於這麼多 RR 就不算頻域 (回傳全 0)

static const char *HRV_FREQ_NAMES[HRV_N_FREQ] = {
    "LF", "HF", "VLF", "LFHF", "total_power", "lfp", "hfp"
};

// 頻帶邊界 (Hz),與 PC 端 FD_BANDS 相同
static constexpr float FD_VLF_LO = 0.003f, FD_VLF_HI = 0.04f;
static constexpr float FD_LF_LO  = 0.04f,  FD_LF_HI  = 0.15f;
static constexpr float FD_HF_LO  = 0.15f,  FD_HF_HI  = 0.4f;

/* 旋轉因子表:angle = -2πk/N,k = 0..N/2-1。用 double 算一次再存成 float,
 * 避免每個蝶形都呼叫三角函數 (256 點共 1024 次,在 MCU 上是可觀的成本)。 */
static float fdTwRe[FD_NFFT / 2], fdTwIm[FD_NFFT / 2];
static bool  fdTwReady = false;

static void fdInitTwiddle()
{
    if (fdTwReady) return;
    for (int k = 0; k < FD_NFFT / 2; k++) {
        double a = -2.0 * M_PI * k / FD_NFFT;
        fdTwRe[k] = (float)cos(a);
        fdTwIm[k] = (float)sin(a);
    }
    fdTwReady = true;
}

/* 就地的疊代式 radix-2 Cooley-Tukey FFT (N = FD_NFFT) */
static void fdFft(float *re, float *im)
{
    const int n = FD_NFFT;
    // 位元反轉重排
    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            float t = re[i]; re[i] = re[j]; re[j] = t;
            t = im[i]; im[i] = im[j]; im[j] = t;
        }
    }
    for (int len = 2; len <= n; len <<= 1) {
        int half = len >> 1, stride = n / len;
        for (int i = 0; i < n; i += len) {
            for (int k = 0; k < half; k++) {
                float wr = fdTwRe[k * stride], wi = fdTwIm[k * stride];
                int a = i + k, b = a + half;
                float xr = re[b] * wr - im[b] * wi;
                float xi = re[b] * wi + im[b] * wr;
                re[b] = re[a] - xr; im[b] = im[a] - xi;
                re[a] += xr;        im[a] += xi;
            }
        }
    }
}

/**
 * 由 RR 序列 (ms) 算出 7 個頻域特徵,寫入 out[HRV_N_FREQ]。
 * 順序:LF, HF, VLF, LFHF, total_power, lfp, hfp
 */
static void hrvFreqFeatures(const float *rr, int n, float *out)
{
    memset(out, 0, sizeof(float) * HRV_N_FREQ);
    if (n < FD_MIN_RR) return;

    // 累積時間 (ms)。PC 端是 np.cumsum(rr),所以 t[0] = rr[0]。
    static float t[HRV_MAX_RR];
    double acc = 0;
    for (int i = 0; i < n; i++) { acc += rr[i]; t[i] = (float)acc; }

    float dur = (t[n - 1] - t[0]) / 1000.0f;      // 秒
    if (dur <= 0) return;

    // 線性內插到 256 點均勻網格 (取代平滑樣條)
    static float re[FD_NFFT], im[FD_NFFT];
    float step = (t[n - 1] - t[0]) / (float)(FD_NFFT - 1);
    int j = 0;
    for (int k = 0; k < FD_NFFT; k++) {
        float tt = t[0] + step * k;
        while (j + 2 < n && t[j + 1] < tt) j++;
        float t0 = t[j], t1 = t[j + 1];
        float f = (t1 > t0) ? (tt - t0) / (t1 - t0) : 0.0f;
        if (f < 0) f = 0; else if (f > 1) f = 1;   // 端點外不外插,與 np.interp 相同
        re[k] = rr[j] + f * (rr[j + 1] - rr[j]);
        im[k] = 0;
    }

    // 去均值:不減掉直流的話,它會遠大於所有頻帶的功率
    double mu = 0;
    for (int k = 0; k < FD_NFFT; k++) mu += re[k];
    mu /= FD_NFFT;
    for (int k = 0; k < FD_NFFT; k++) re[k] -= (float)mu;

    fdInitTwiddle();
    fdFft(re, im);

    float fsEff = (FD_NFFT - 1) / dur;             // 網格的等效取樣率
    float df = fsEff / FD_NFFT;
    const float inv = 1.0f / FD_NFFT;

    float vlf = 0, lf = 0, hf = 0;
    for (int k = 0; k <= FD_NFFT / 2; k++) {       // 對應 np.fft.rfft 的頻率格
        float xr = re[k] * inv, xi = im[k] * inv;
        float p = xr * xr + xi * xi;
        float fq = k * df;
        if (fq >= FD_VLF_LO && fq < FD_VLF_HI) vlf += p;
        if (fq >= FD_LF_LO  && fq < FD_LF_HI)  lf  += p;
        if (fq >= FD_HF_LO  && fq < FD_HF_HI)  hf  += p;
    }

    float total = vlf + lf + hf;
    out[0] = lf;
    out[1] = hf;
    out[2] = vlf;
    out[3] = (hf > 0) ? lf / hf : 0.0f;
    out[4] = total;
    out[5] = (total > 0) ? lf / total : 0.0f;
    out[6] = (total > 0) ? hf / total : 0.0f;
}
