/**
 * HRV 時域特徵 —— 與 PC 端 preprocessing_tool/feature_extraction.py 逐行對應
 *
 * 只實作 11 個時域特徵。頻域特徵 (LF/HF/VLF/...) 與 D2 沒有實作,不是效能問題,
 * 而是無法在 MCU 上做到數值一致:
 *   - 頻域要 scipy UnivariateSpline(k=3) 的 FITPACK 平滑樣條 (節點選擇 + 平滑度
 *     求解) 再對 6 萬點做 FFT;那條樣條同時也是訓練資料裡 1e48 發散值的來源。
 *   - D2 用 nolds.corr_dim 的 RANSAC 擬合,本身就有隨機性。
 * 實測消融顯示這些特徵對準確率沒有貢獻 (LOSO:11 維 92.2% vs 25 維 91.8%),
 * 因此部署版直接採用 11 維時域模型。
 *
 * 特徵順序必須與 tools/stress_model.h 的 STRESS_FEAT_NAMES 完全相同。
 */
#pragma once

#include <math.h>
#include <string.h>

#define HRV_MAX_RR 512
#include "hrv_freq.h"   // 頻域特徵 (線性內插 + 256 點 FFT)

#define HRV_N_TD   11
#define HRV_N_FEAT (HRV_N_TD + HRV_N_FREQ)   // 11 時域 + 7 頻域
#define HRV_MAX_RR 512

// 視窗長度 (秒)。必須與 tools/extract_wesad.py 的 WINDOW_SEC 相同 —— 特徵是
// 在這個長度的視窗上算出來的,長度不同分布就對不上模型權重。
// main.cpp 的 STRESS_WIN 也由這個常數推導,避免兩處各改一半。
#define HRV_WINDOW_SEC 120
#define HRV_FS         64        // 處理鏈的取樣率 (WESAD 的 E4 BVP)

// 峰值偵測參數。必須與 tools/extract_wesad.py 的 PROMINENCE / MIN_DIST_MS 相同,
// 否則兩端抓到的心跳不一樣,特徵就不是同一回事。
// 450 ms 是擋重搏波的主力:它緊跟在收縮峰後約 200-350 ms,而 scipy 的 distance
// 篩選保留較高的峰,正好把它刷掉。300 ms 擋不住 (實機心率被高估成 151 bpm)。
#define HRV_PEAK_PROM    0.3f
#define HRV_PEAK_DIST_MS 400

// pNN20/pNN50 的分母。PC 端是 window_length = fs * WINDOW_SEC (樣本數,不是心跳數
// —— 這是原作者的定義)。手錶感測器是 100Hz,但這個分母必須用處理鏈的 64Hz 與
// 視窗長度算,否則特徵尺度會差 100/64 = 1.5625 倍,標準化之後就對不上權重。
static constexpr float HRV_PNN_DENOM = (float)(HRV_FS * HRV_WINDOW_SEC);

// 峰數下限,必須與 tools/extract_wesad.py 的 MIN_PEAK 相同。
// 0 = 不做品質篩選;此時只剩數學上的最低要求 (篩選後至少 4 個 RR)。
static constexpr int HRV_MIN_PEAK = 0;

struct HrvResult {
    bool  valid;
    int   nPeak;                 // 偵測到的峰數
    int   nRR;                   // 經 mean±300ms 篩選後的 RR 數
    float feat[HRV_N_FEAT];
    float rr[HRV_MAX_RR];        // 篩選後的 RR (ms),供比對驗證用
};

// 順序必須與 tools/extract_wesad.py 的 FEAT_NAMES 完全相同
static const char *HRV_FEAT_NAMES[HRV_N_FEAT] = {
    "HR_mean", "HR_std", "meanNN", "SDNN", "medianNN",
    "meanSD", "SDSD", "RMSSD", "pNN20", "pNN50", "TINN",
    "LF", "HF", "VLF", "LFHF", "total_power", "lfp", "hfp"
};

// ---------- 小工具 (全部對應 numpy 的預設行為) ----------

static float hrvMean(const float *v, int n)
{
    if (n <= 0) return 0;
    double s = 0;                       // 用 double 累加,避免長序列的累積誤差
    for (int i = 0; i < n; i++) s += v[i];
    return (float)(s / n);
}

/* np.std 預設 ddof=0 (母體標準差) */
/* 中位數:quickselect (平均 O(n)),會打亂 a[] 的順序,呼叫端傳暫存副本。
 * n 為偶數時取中間兩個的平均,與 numpy.median 相同。 */
static float hrvSelectK(float *a, int n, int k)
{
    int lo = 0, hi = n - 1;
    while (lo < hi) {
        float pv = a[(lo + hi) / 2]; int i = lo, j = hi;
        while (i <= j) {
            while (a[i] < pv) i++;
            while (a[j] > pv) j--;
            if (i <= j) { float t = a[i]; a[i] = a[j]; a[j] = t; i++; j--; }
        }
        if (k <= j) hi = j; else if (k >= i) lo = i; else break;
    }
    return a[k];
}

static float hrvMedianInPlace(float *a, int n)
{
    if (n <= 0) return 0;
    float hi = hrvSelectK(a, n, n / 2);
    if (n & 1) return hi;
    float lo = a[0];                          // 選完後 [0, n/2) 都 <= a[n/2],找其中最大
    for (int i = 1; i < n / 2; i++) if (a[i] > lo) lo = a[i];
    return 0.5f * (lo + hi);
}

/* 視窗正規化:中位數 / MAD (×1.4826 換算成 σ 尺度)。與 PC 端 extract_wesad.normalize(NORM="mad")
 * 相同。用 MAD 而不用標準差:一次 0.3 秒的晃動尖峰會把標準差撐大好幾倍,讓整個視窗的心跳
 * 突出度掉到門檻以下;中位數與 MAD 幾乎不受不到 1% 樣本的離群值影響。
 * 回傳 false 表示訊號平坦 (MAD≈0)。scratch 需可容納 n 個 float。 */
#define HRV_MAD_K 1.4826f
static bool hrvNormalizeMad(float *w, int n, float *scratch)
{
    for (int i = 0; i < n; i++) scratch[i] = w[i];
    float med = hrvMedianInPlace(scratch, n);
    for (int i = 0; i < n; i++) scratch[i] = fabsf(w[i] - med);
    float mad = hrvMedianInPlace(scratch, n) * HRV_MAD_K;
    if (mad <= 1e-9f) return false;
    float inv = 1.0f / mad;
    for (int i = 0; i < n; i++) w[i] = (w[i] - med) * inv;
    return true;
}

static float hrvStd(const float *v, int n)
{
    if (n <= 0) return 0;
    double m = 0;
    for (int i = 0; i < n; i++) m += v[i];
    m /= n;
    double s = 0;
    for (int i = 0; i < n; i++) { double d = v[i] - m; s += d * d; }
    return (float)sqrt(s / n);
}

/* np.median:偶數長度取中間兩個的平均。會就地排序傳入的陣列。 */
static float hrvMedian(float *v, int n)
{
    if (n <= 0) return 0;
    for (int i = 1; i < n; i++) {       // 插入排序,n 只有數十筆
        float k = v[i];
        int j = i - 1;
        while (j >= 0 && v[j] > k) { v[j + 1] = v[j]; j--; }
        v[j + 1] = k;
    }
    return (n & 1) ? v[n / 2] : 0.5f * (v[n / 2 - 1] + v[n / 2]);
}

/* calc_heartrate():RR (ms) → 瞬時心率,異常值以最近 10 筆的平均補上 */
static void hrvHeartRate(const float *rr, int n, float *hr)
{
    const int W = 10;
    for (int i = 0; i < n; i++) {
        float val = rr[i];
        if (val > 400.0f && val < 1500.0f) {
            hr[i] = 60000.0f / val;
        } else if ((val > 0.0f && val < 400.0f) || val > 1500.0f) {
            if (i > 0) {
                int lo = i - W; if (lo < 0) lo = 0;   // HR[-10:]
                hr[i] = hrvMean(&hr[lo], i - lo);
            } else {
                hr[i] = 60.0f;
            }
        } else {
            hr[i] = 0.0f;                              // val == 400 或 1500 剛好落這裡
        }
    }
}

/**
 * 由峰值索引算出 11 個時域特徵。
 * peakIdx 為訊號中的樣本索引,fs 為該訊號的取樣率 (Hz)。
 *
 * 對應 calc_RRI() + calc_td_hrv()。
 */
static bool hrvFromPeaks(const int *peakIdx, int nPeak, float fs, HrvResult *out)
{
    memset(out, 0, sizeof(*out));
    out->nPeak = nPeak;
    if (nPeak < HRV_MIN_PEAK) return false;

    // --- calc_RRI:峰間距 → ms ---
    static float rrAll[HRV_MAX_RR];
    int nAll = 0;
    for (int i = 0; i + 1 < nPeak && nAll < HRV_MAX_RR; i++)
        rrAll[nAll++] = (peakIdx[i + 1] - peakIdx[i]) * 1000.0f / fs;
    if (nAll < 1) return false;

    // 只保留落在 mean±300ms 內的 RR (原始碼的異常心跳剔除)
    float meanRR = hrvMean(rrAll, nAll);
    float *rrE = out->rr;
    int nE = 0;
    for (int i = 0; i < nAll; i++)
        if (rrAll[i] > meanRR - 300.0f && rrAll[i] < meanRR + 300.0f)
            rrE[nE++] = rrAll[i];
    out->nRR = nE;
    if (nE <= 3) return false;          // 對應 len(RR_list) <= 3 的提前返回

    // 逐差:注意原始碼取的是絕對值,平方差則不取
    static float rrDiff[HRV_MAX_RR], rrSq[HRV_MAX_RR];
    int nD = 0;
    for (int i = 0; i + 1 < nE; i++) {
        float d = rrE[i] - rrE[i + 1];
        rrDiff[nD] = fabsf(d);
        rrSq[nD] = d * d;
        nD++;
    }
    if (nD < 1) return false;

    // --- calc_td_hrv ---
    static float hr[HRV_MAX_RR];
    hrvHeartRate(rrE, nE, hr);

    static float tmp[HRV_MAX_RR];

    memcpy(tmp, rrE, nE * sizeof(float));
    for (int i = 0; i < nE; i++) tmp[i] = fabsf(tmp[i]);   // np.median(np.abs(RR))
    float medianNN = hrvMedian(tmp, nE);

    int nn20 = 0, nn50 = 0;
    for (int i = 0; i < nD; i++) {
        if (rrDiff[i] > 20.0f) nn20++;
        if (rrDiff[i] > 50.0f) nn50++;
    }

    // TINN = np.histogram 的邊界跨度。預設 10 個 bin,邊界從 min 到 max,
    // 所以 max(bar_x)-min(bar_x) 就等於 max(RR)-min(RR)。
    // 全部相等時 numpy 會改用 [v-0.5, v+0.5],跨度為 1。
    float mn = rrE[0], mx = rrE[0];
    for (int i = 1; i < nE; i++) { if (rrE[i] < mn) mn = rrE[i]; if (rrE[i] > mx) mx = rrE[i]; }
    float TINN = (mx > mn) ? (mx - mn) : 1.0f;

    float *f = out->feat;
    f[0]  = hrvMean(hr, nE);                    // HR_mean
    f[1]  = hrvStd(hr, nE);                     // HR_std
    f[2]  = hrvMean(rrE, nE);                   // meanNN
    f[3]  = hrvStd(rrE, nE);                    // SDNN
    f[4]  = medianNN;                           // medianNN
    f[5]  = hrvMean(rrDiff, nD);                // meanSD
    f[6]  = hrvStd(rrDiff, nD);                 // SDSD
    f[7]  = sqrtf(hrvMean(rrSq, nD));           // RMSSD
    f[8]  = nn20 / HRV_PNN_DENOM;               // pNN20
    f[9]  = nn50 / HRV_PNN_DENOM;               // pNN50
    f[10] = TINN;

    hrvFreqFeatures(rrE, nE, &f[HRV_N_TD]);   // 頻域接在時域之後

    out->valid = true;
    return true;
}
