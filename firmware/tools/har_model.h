// 活動辨識模型 (LDA) — 由 tools/train_har.py 產生 (WISDM 手錶, LOSO)
// 推論:x=clip(x,LO,HI) → z=(x-MEAN)/SCALE → score_c = dot(W[c],z)+B[c]
//       → 類別 = argmax_c score_c
#pragma once

#define HAR_N_FEAT 10
#define HAR_N_CLASS 4

static const char *HAR_CLASS_NAMES[HAR_N_CLASS] = {"sit", "stand", "walk", "run"};
static const char *HAR_FEAT_NAMES[HAR_N_FEAT] = {
    "acc_mean_x", "acc_mean_y", "acc_mean_z", "acc_std_x", "acc_std_y", "acc_std_z", "acc_mag_mean", "acc_mag_std", "acc_mag_min", "acc_mag_max"
};

static const float HAR_CLIP_LO[HAR_N_FEAT] = {
    -11.332503f, -12.226902f, -8.2921109f, 0.014387292f, 0.012140796f, 0.01338907f,
    9.2292617f, 0.011594817f, 0.99728367f, 9.3489223f,
};

static const float HAR_CLIP_HI[HAR_N_FEAT] = {
    12.940459f, 8.3995147f, 9.7393069f, 11.995764f, 14.402439f, 10.97596f,
    21.923804f, 12.186819f, 11.022387f, 51.162455f,
};

static const float HAR_MEAN[HAR_N_FEAT] = {
    3.3837902f, -3.9943384f, 1.6091774f, 2.7915005f, 2.9930402f, 1.7797031f,
    11.329084f, 2.3647146f, 7.5597232f, 16.489793f,
};

static const float HAR_SCALE[HAR_N_FEAT] = {
    5.8831927f, 3.6915075f, 4.3090996f, 3.3589867f, 3.6905918f, 2.2066035f,
    2.388839f, 2.6999624f, 2.360928f, 8.1030405f,
};

static const float HAR_W[HAR_N_CLASS][HAR_N_FEAT] = {
    {-0.71281112f, -0.18882814f, 1.5648144f, -2.6588747f, -2.7934504f, 0.67766565f, -1.3669578f, -9.2449783f, 0.80453354f, 6.5424312f},
    {0.20872386f, 0.16913051f, -0.52256131f, -2.6677968f, -2.7624314f, 0.88002295f, -2.2036064f, -9.538327f, 0.6984499f, 7.308191f},
    {0.6705384f, 0.42875566f, -0.64951558f, -0.514231f, -5.2590237f, -0.78169643f, 3.0947991f, -1.156126f, -3.0768072f, 2.2889655f},
    {-0.16560569f, -0.41385657f, -0.41104348f, 5.9043645f, 10.975955f, -0.77391155f, 0.44120827f, 20.147453f, 1.6192692f, -16.316216f},
};
static const float HAR_B[HAR_N_CLASS] = {-6.0729755f, -5.1809906f, -2.8248865f, -17.113645f};
