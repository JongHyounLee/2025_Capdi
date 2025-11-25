import os
import json
import numpy as np
import pandas as pd

# ==============================
# 설정 부분 (지민 PC에 맞게 수정)
# ==============================
BASE = r"C:\Users\hjl99\Desktop\IMU_MODEL_DATA"

# 학습 때 만든 정규화 정보 JSON
STATS_JSON = os.path.join(BASE, "tcn_norm_stats.json")

# ==============================
#  유틸 함수들
# ==============================
def load_norm_stats(json_path):
    """
    tcn_norm_stats.json에서
    - L (시퀀스 길이)
    - channels (60채널 이름)
    - mu, sigma (각 60개)
    를 읽어서 반환
    """
    with open(json_path, "r", encoding="utf-8") as f:
        d = json.load(f)

    L = int(d["L"])
    channels = d["channels"]
    mu = np.array(d["mu"], dtype=np.float32)
    sigma = np.array(d["sigma"], dtype=np.float32)

    if len(channels) != len(mu) or len(mu) != len(sigma):
        raise ValueError("stats 파일의 channels / mu / sigma 길이가 맞지 않습니다.")

    sigma[sigma < 1e-6] = 1.0  # 0으로 나누기 방지
    return L, channels, mu, sigma


def _pick_cols(df: pd.DataFrame, channels):
    """
    - *_tick, Timestep 제거
    - move / label 이름 맞추기
    - stats.json에서 읽은 channels 순서대로 60채널 추출
    - 마지막에 move, label 붙여서 반환
    """
    # 1) tick, timestep 열 제거
    df = df[[c for c in df.columns
             if "_tick" not in c.lower()
             and c.lower() != "timestep"]]

    # 2) 소문자 매핑
    low = {c.lower(): c for c in df.columns}

    # 3) move 표준화
    if "move" not in df.columns:
        mkey = next((k for k in low if k == "move"), None)
        if mkey is None:
            # move 없으면 전체를 move=1 하나로 처리
            df["move"] = 1
        else:
            df.rename(columns={low[mkey]: "move"}, inplace=True)

    # 4) label 표준화
    if "label" not in df.columns:
        lkey = next((k for k in low if k == "label"), None)
        if lkey is None:
            df["label"] = -1   # 없으면 -1로 채움
        else:
            df.rename(columns={low[lkey]: "label"}, inplace=True)

    # 다시 low 갱신
    low = {c.lower(): c for c in df.columns}

    cols = []
    for c in channels:
        key = c.lower()
        if key not in low:
            raise ValueError(f"필수 채널 누락: {c}")
        cols.append(low[key])

    return df[cols + ["move", "label"]]


def _resample_to_L(x_T_C: np.ndarray, L: int) -> np.ndarray:
    """
    x: [T, C] → 선형 보간으로 [L, C]
    (학습 코드의 _resample_to_L와 동일)
    """
    T, C = x_T_C.shape
    if T < 2:
        return np.repeat(x_T_C, L, axis=0)

    t_src = np.linspace(0.0, 1.0, num=T, dtype=np.float32)
    t_dst = np.linspace(0.0, 1.0, num=L, dtype=np.float32)
    out = np.empty((L, C), dtype=np.float32)

    for c in range(C):
        out[:, c] = np.interp(t_dst, t_src, x_T_C[:, c])

    return out


def preprocess_csv(input_path, stats_json=STATS_JSON, out_suffix="_preproc_60ch.csv"):
    """
    1) tcn_norm_stats.json 로드 (L, channels, mu, sigma)
    2) 입력 CSV에서 60채널 + move + label 추출
    3) move별로 잘라서 [T,60] → 리샘플 [L,60]
    4) mu/sigma로 z-score 정규화
    5) 각 move 시퀀스를 한 CSV로 이어붙여서 저장
       - Timestep: 각 move마다 0~L-1로 리셋
       - move   : 원래 move 번호 유지
       - label  : 해당 move 구간의 다수결 라벨
    """
    print("\n[INFO] ==== 전처리 시작 ====")
    print("[INFO] 입력 CSV:", input_path)
    print("[INFO] stats json:", stats_json)

    L, channels, mu, sigma = load_norm_stats(stats_json)
    print(f"[INFO] L={L}, 채널 수={len(channels)}")

    df_raw = pd.read_csv(input_path)
    print("[INFO] 원본 df shape:", df_raw.shape)

    df = _pick_cols(df_raw, channels)   # 60채널 + move + label
    print("[INFO] 사용 df 컬럼:", df.columns.tolist())

    all_rows = []
    move_values = df["move"].unique()
    move_values = np.sort(move_values)

    print(f"[INFO] 감지된 move 개수: {len(move_values)} → {move_values}")

    for mv in move_values:
        g = df[df["move"] == mv]

        # [T,60] raw
        x = g[channels].astype(np.float32).values
        T_cur = x.shape[0]
        if T_cur == 0:
            continue

        # [L,60] 리샘플
        x_rs = _resample_to_L(x, L=L)

        # 정규화 (mu/sigma)
        x_norm = (x_rs - mu) / sigma  # [L,60]

        # label 다수결
        lbl_vals = g["label"].astype(int).values
        lbl = int(np.round(lbl_vals.mean()))

        # DataFrame으로 쌓기
        df_mv = pd.DataFrame(x_norm, columns=channels)
        df_mv.insert(0, "Timestep", np.arange(L, dtype=np.int32))  # 0~L-1
        df_mv["move"] = int(mv)
        df_mv["label"] = lbl

        all_rows.append(df_mv)

        print(f"  - move {mv}: 원본 T={T_cur} → 리샘플 L={L}, label={lbl}")

    if not all_rows:
        raise RuntimeError("move별로 생성된 시퀀스가 없습니다.")

    df_out = pd.concat(all_rows, axis=0, ignore_index=True)
    print("[INFO] 최종 전처리 df shape:", df_out.shape)

    base, ext = os.path.splitext(input_path)
    out_path = base + out_suffix
    df_out.to_csv(out_path, index=False, encoding="utf-8-sig")

    print("✅ 전처리 CSV 저장 완료:", out_path)
    return out_path


# ==============================
#  메인
# ==============================
if __name__ == "__main__":
    # 테스트용: 여기다가 처리할 CSV 경로 넣어두고 실행
    # 예) imu_label_0_ALL_feat.csv 전체에 전처리 적용
    TEST_CSV = os.path.join(BASE, "imu_label_0_ALL_feat.csv")

    # 필요할 때 이 라인만 바꿔서 다른 CSV에도 반복 사용하면 됨
    preprocess_csv(TEST_CSV)
