#pragma once
/* STFT ACC 遮罩 (M9a):把 ACC 主頻與 2 倍頻附近 (±0.10 Hz) 的 PPG 頻譜壓到 0.1 倍,再反 STFT 回波形。
 * 只在幀內 |a| 標準差 > 0.08 g (真的在走動) 且 ACC 頻譜有明顯峰 (>5× 中位數) 時才遮。
 * 與 tools/stft_mask_ref.py 一對一 (Hann 512、hop 64、鏡射延拓、補零到整數幀、w² 正規化),
 * 兩端數值必須一致;沒有遮罩時是恆等重建。
 * 記憶體:延拓後的訊號與輸出各 (7680+512+512) floats,放 PSRAM (呼叫端提供)。 */
#include <math.h>
#include <string.h>

namespace stftmask {

static constexpr int   FS = 64, NPER = 512, HOP = 64, PAD = NPER / 2;
static constexpr float TOL = 0.10f, GAIN = 0.1f, F_LO = 0.5f, F_HI = 3.5f;
static constexpr float ACC_MIN_STD = 0.08f;    // g
static constexpr int   LOG2N = 9;

static float twRe[NPER / 2], twIm[NPER / 2], win[NPER];
static bool  inited = false;

static void init()
{
    if (inited) return;
    for (int k = 0; k < NPER / 2; k++) {
        double a = -2.0 * M_PI * k / NPER; twRe[k] = (float)cos(a); twIm[k] = (float)sin(a);
    }
    for (int i = 0; i < NPER; i++) win[i] = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * i / NPER);   // 週期性 Hann
    inited = true;
}

/* 就地 radix-2 複數 FFT (N = 512)。inverse=true 時用共軛並除以 N。 */
static void fft(float *re, float *im, bool inverse)
{
    const int n = NPER;
    for (int i = 1, j = 0; i < n; i++) {                       // 位元反轉
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) { float t = re[i]; re[i] = re[j]; re[j] = t; t = im[i]; im[i] = im[j]; im[j] = t; }
    }
    for (int len = 2; len <= n; len <<= 1) {
        int step = n / len;
        for (int i = 0; i < n; i += len) {
            for (int k = 0; k < len / 2; k++) {
                float wr = twRe[k * step], wi = inverse ? -twIm[k * step] : twIm[k * step];
                int a = i + k, b = i + k + len / 2;
                float xr = re[b] * wr - im[b] * wi, xi = re[b] * wi + im[b] * wr;
                re[b] = re[a] - xr; im[b] = im[a] - xi; re[a] += xr; im[a] += xi;
            }
        }
    }
    if (inverse) { float s = 1.0f / n; for (int i = 0; i < n; i++) { re[i] *= s; im[i] *= s; } }
}

/* ACC 幀 (NPER×3, g) 的主頻:三軸各自去均值 FFT,0.5–3.5 Hz 內最大且 >5× 中位數,拋物線內插。
 * 回 <0 表示沒有動作或沒有明顯峰。 */
static float accPeak(const float *ax, const float *ay, const float *az, int n)
{
    static float re[NPER], im[NPER], P[NPER / 2 + 1], tmp[NPER / 2 + 1];
    // 動作能量閘:|a| 的標準差
    double mu = 0, s2 = 0;
    for (int i = 0; i < n; i++) { float m = sqrtf(ax[i] * ax[i] + ay[i] * ay[i] + az[i] * az[i]); mu += m; s2 += (double)m * m; }
    mu /= n; double var = s2 / n - mu * mu;
    if (var < (double)ACC_MIN_STD * ACC_MIN_STD) return -1.0f;
    int kLo = (int)ceilf(F_LO * NPER / FS), kHi = (int)floorf(F_HI * NPER / FS);
    float best = -1.0f, bestP = 0.0f;
    const float *axes[3] = {ax, ay, az};
    for (int a = 0; a < 3; a++) {
        double m = 0; for (int i = 0; i < n; i++) m += axes[a][i]; m /= n;
        for (int i = 0; i < NPER; i++) { re[i] = i < n ? (float)(axes[a][i] - m) : 0.0f; im[i] = 0.0f; }
        fft(re, im, false);
        for (int k = 0; k <= NPER / 2; k++) P[k] = re[k] * re[k] + im[k] * im[k];
        int nb = kHi - kLo + 1;
        for (int k = 0; k < nb; k++) tmp[k] = P[kLo + k];
        // 中位數 (nb 小,插入排序)
        for (int i = 1; i < nb; i++) { float v = tmp[i]; int j = i - 1; while (j >= 0 && tmp[j] > v) { tmp[j + 1] = tmp[j]; j--; } tmp[j + 1] = v; }
        float med = (nb & 1) ? tmp[nb / 2] : 0.5f * (tmp[nb / 2 - 1] + tmp[nb / 2]);
        int gi = kLo; for (int k = kLo; k <= kHi; k++) if (P[k] > P[gi]) gi = k;
        float pk = P[gi];
        if (pk > bestP && pk > 5.0f * med) {
            float d = 0.0f;
            if (gi > 0 && gi < NPER / 2) {
                float pa = P[gi - 1], pb = P[gi], pc = P[gi + 1], den = pa - 2 * pb + pc;
                if (den != 0.0f) d = 0.5f * (pa - pc) / den;
            }
            bestP = pk; best = (gi + d) * (float)FS / NPER;
        }
    }
    return best;
}

/* bp[n]:帶通後 PPG (64 Hz);acc[n][3]:對齊到 64 Hz 的加速度 (g)。結果寫回 bp。
 * ext, out, norm:呼叫端提供的暫存,長度 >= n + NPER + NPER (延拓 + 補零)。 */
static void apply(float *bp, const float (*acc)[3], int n, float *ext, float *out, float *norm)
{
    init();
    static float re[NPER], im[NPER];
    static float fax[NPER], fay[NPER], faz[NPER];
    // 鏡射延拓 (scipy boundary='even') + 補零到整數幀
    for (int i = 0; i < PAD; i++) ext[i] = bp[PAD - i];
    memcpy(ext + PAD, bp, sizeof(float) * n);
    for (int i = 0; i < PAD; i++) ext[PAD + n + i] = bp[n - 2 - i];
    int lenE = n + 2 * PAD;
    int nfr = (lenE - NPER + HOP - 1) / HOP + 1;
    int total = (nfr - 1) * HOP + NPER;
    for (int i = lenE; i < total; i++) ext[i] = 0.0f;
    memset(out, 0, sizeof(float) * total); memset(norm, 0, sizeof(float) * total);
    const float df = (float)FS / NPER;
    for (int j = 0; j < nfr; j++) {
        int s = j * HOP;
        for (int i = 0; i < NPER; i++) { re[i] = ext[s + i] * win[i]; im[i] = 0.0f; }
        fft(re, im, false);
        // ACC 幀:幀中心 ±NPER/2 (原始座標),不足補零
        int c = s + PAD - PAD;                       // 幀起點在原始座標 = s - PAD + PAD... 見下
        c = s + NPER / 2 - PAD;                      // 幀中心
        int a0 = c - NPER / 2, a1 = c + NPER / 2; if (a0 < 0) a0 = 0; if (a1 > n) a1 = n;
        if (a1 - a0 >= NPER / 2) {
            int m = a1 - a0;
            for (int i = 0; i < NPER; i++) {
                if (i < m) { fax[i] = acc[a0 + i][0]; fay[i] = acc[a0 + i][1]; faz[i] = acc[a0 + i][2]; }
                else { fax[i] = fay[i] = faz[i] = 0.0f; }
            }
            float fa = accPeak(fax, fay, faz, m == NPER ? NPER : NPER);   // 補零後長度 NPER (與鏡像相同)
            if (fa > 0.0f) {
                for (int h = 1; h <= 2; h++) {
                    float fc = h * fa;
                    for (int k = 0; k <= NPER / 2; k++) {
                        if (fabsf(k * df - fc) <= TOL) {
                            re[k] *= GAIN; im[k] *= GAIN;
                            if (k > 0 && k < NPER / 2) { re[NPER - k] *= GAIN; im[NPER - k] *= GAIN; }   // 共軛對稱
                        }
                    }
                }
            }
        }
        fft(re, im, true);
        for (int i = 0; i < NPER; i++) { out[s + i] += re[i] * win[i]; norm[s + i] += win[i] * win[i]; }
    }
    for (int i = 0; i < n; i++) { float d = norm[PAD + i]; bp[i] = d > 1e-10f ? out[PAD + i] / d : out[PAD + i]; }
}

} // namespace stftmask
