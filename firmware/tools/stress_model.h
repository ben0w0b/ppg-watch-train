// WESAD 壓力二分類模型 (LogisticRegression) — 由 tools/train_stress.py 產生
// 推論:x=clip(x,CLIP_LO,CLIP_HI) → z=(x-MEAN)/SCALE
//       → score=dot(W,z)+B → p=1/(1+exp(-score))
// score > 0 判為壓力 (label=1)
#pragma once

#define STRESS_N_FEAT 18

static const char *STRESS_FEAT_NAMES[] = {
    "HR_mean", "HR_std", "meanNN", "SDNN", "medianNN", "meanSD", "SDSD", "RMSSD", "pNN20", "pNN50", "TINN", "LF", "HF", "VLF", "LFHF", "total_power", "lfp", "hfp"
};

static const float STRESS_CLIP_LO[STRESS_N_FEAT] = {
    54.617915f, 4.483179f, 457.17046f, 34.982846f,
    437.5f, 21.168529f, 22.588606f, 34.785207f,
    0.0078125f, 0.0018652344f, 228.90625f, 150.01673f,
    81.115768f, 102.84389f, 0.24062429f, 482.03902f,
    0.13837614f, 0.095804876f,
};

static const float STRESS_CLIP_HI[STRESS_N_FEAT] = {
    130.7244f, 25.453379f, 1117.2408f, 182.49138f,
    1140.625f, 210.52445f, 153.41283f, 255.56303f,
    0.019401042f, 0.016927083f, 593.75f, 4389.5764f,
    4539.5164f, 5693.6843f, 4.3955326f, 11548.604f,
    0.6338317f, 0.71351125f,
};

static const float STRESS_MEAN[STRESS_N_FEAT] = {
    82.312322f, 13.161123f, 773.62543f, 113.95022f,
    772.88242f, 100.34875f, 98.913188f, 141.73533f,
    0.013135355f, 0.0089188054f, 537.63377f, 1429.6872f,
    1680.5775f, 992.35466f, 1.0387986f, 4108.5495f,
    0.35513722f, 0.40267752f,
};

static const float STRESS_SCALE[STRESS_N_FEAT] = {
    16.136345f, 5.2230654f, 138.78621f, 30.959168f,
    153.19139f, 42.303356f, 27.574054f, 48.081974f,
    0.0025364026f, 0.003246069f, 58.804095f, 778.1407f,
    992.20088f, 882.69207f, 0.63973415f, 2033.873f,
    0.097770312f, 0.12771257f,
};

static const float STRESS_W[STRESS_N_FEAT] = {
    1.4165332f, -1.099943f, -1.3784251f, 0.0003909362f,
    -1.4803333f, 3.7661236f, -3.3293871f, 1.0570812f,
    1.3359436f, 1.8096567f, -0.028584492f, -0.94881315f,
    1.3751742f, -0.65582736f, -0.24797669f, 0.041183068f,
    0.8607678f, -1.4735637f,
};

static const float STRESS_B = -2.00863f;
