// 由 tools/extract_wesad.py 自動產生 —— 請勿手動修改
// scipy.signal.butter(2, [0.5/32.0, 10.0/32.0], btype='band', output='sos')
// 帶通 0.5-10.0 Hz, order 2, fs = 64
#pragma once

static constexpr int PPG_NSOS = 2;
// 每級為 {b0, b1, b2, a1, a2} (a0 已正規化為 1)
static const float PPG_SOS[PPG_NSOS][5] = {
    { 0.12892624f, 0.257852479f, 0.12892624f, -0.757350633f, 0.295163671f },
    { 1.0f, -2.0f, 1.0f, -1.93100386f, 0.933543199f },
};
