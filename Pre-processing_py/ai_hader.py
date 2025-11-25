# === Generate C preprocessing files from stats JSON (60채널 대응 버전) ===
import json, os

BASE = "C:\\Users\\hjl99\\Desktop\\IMU_MODEL_DATA"   # 학습에 사용한 폴더와 동일하게

# --- 1) JSON 읽기 ---
with open(f"{BASE}/tcn_norm_stats_90ch.json", "r", encoding="utf-8") as f:
    stats = json.load(f)

# JSON 안에 들어있는 값 사용
L = int(stats.get("L", 128))     # 기본값 128
mu = stats["mu"]
sigma = stats["sigma"]

C = len(mu)                      # 채널 수 = mu 길이 (지금은 60이어야 함)
assert len(sigma) == C, "mu/sigma 길이가 다릅니다!"
if "channels" in stats:
    assert len(stats["channels"]) == C, "channels 길이와 mu 길이가 다릅니다!"

print(f"[INFO] L={L}, C={C}, len(mu)={len(mu)}, len(sigma)={len(sigma)}")

def fmt(arr):
    lines = []
    for x in arr:
        lines.append(f"{float(x):.6f}f")
    # 보기 좋게 줄바꿈
    return ",\n  ".join(lines)

# --- 2) 헤더 파일 내용 ---
h = f"""#ifndef CUBEAI_PREPROCESS_H
#define CUBEAI_PREPROCESS_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {{
#endif

// --- TCN 입력 크기 (JSON에서 읽어옴) ---
#define TCN_L {L}
#define TCN_C {C}

// 선형 리샘플: src [src_len, C] → dst [L, C]
void tcn_linear_resample(const float* src, int src_len, int C, float* dst, int L);

// 채널별 z-score 정규화 (μ/σ)
void tcn_normalize_inplace(float* x, int L, int C);

// 디버깅용 getter
const float* tcn_get_mu(void);
const float* tcn_get_sigma(void);

#ifdef __cplusplus
}}
#endif
#endif // CUBEAI_PREPROCESS_H
"""

# --- 3) C 파일 내용 ---
c = f"""#include "cubeai_preprocess.h"
#include <math.h>
#include <stddef.h>

// 채널별 평균/표준편차 (길이 TCN_C = {C})
static const float kMu[TCN_C] = {{
  {fmt(mu)}
}};

static const float kSigma[TCN_C] = {{
  {fmt(sigma)}
}};

// src: [src_len, C] row-major → dst: [L, C]
void tcn_linear_resample(const float* src, int src_len, int C, float* dst, int L) {{
    if (!src || !dst || src_len <= 0 || L <= 0 || C <= 0) return;

    for (int i = 0; i < L; ++i) {{
        float pos = (float)(src_len - 1) * (float)i / (float)(L - 1);
        int i0 = (int)pos;
        int i1 = i0 + 1;
        if (i0 < 0) i0 = 0;
        if (i1 >= src_len) i1 = src_len - 1;
        float t = pos - (float)i0;

        const float* s0 = src + (size_t)i0 * C;
        const float* s1 = src + (size_t)i1 * C;
        float* d = dst + (size_t)i * C;

        for (int c = 0; c < C; ++c) {{
            d[c] = s0[c] * (1.0f - t) + s1[c] * t;
        }}
    }}
}}

// x: [L, C] row-major
void tcn_normalize_inplace(float* x, int L, int C) {{
    if (!x || L <= 0 || C <= 0) return;

    const float eps = 1e-6f;
    for (int i = 0; i < L; ++i) {{
        float* row = x + (size_t)i * C;
        for (int c = 0; c < C; ++c) {{
            float s = kSigma[c];
            if (s < eps) s = 1.0f;
            row[c] = (row[c] - kMu[c]) / s;
        }}
    }}
}}

const float* tcn_get_mu(void)    {{ return kMu;    }}
const float* tcn_get_sigma(void) {{ return kSigma; }}
"""

# --- 4) 파일로 저장 ---
with open(f"{BASE}/cubeai_preprocess.h", "w", encoding="utf-8") as f:
    f.write(h)
with open(f"{BASE}/cubeai_preprocess.c", "w", encoding="utf-8") as f:
    f.write(c)

print("Created:")
print(f" - {BASE}/cubeai_preprocess.h")
print(f" - {BASE}/cubeai_preprocess.c")
