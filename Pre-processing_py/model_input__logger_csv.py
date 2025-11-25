import pandas as pd
import tkinter as tk
from tkinter import filedialog
import os
import re

def build_90ch_column_names():
    cols = ["Timestep"]  # t → Timestep

    # IMU당 16채널: 6(raw) + 6(feature) + 4(quat)
    per_imu_feats = [
        "ax", "ay", "az",
        "gx", "gy", "gz",
        "a_mag", "g_mag",
        "pitch", "roll",
        "jerk_a_mag", "yaw_int",
        "qw", "qx", "qy", "qz",
    ]

    # IMU1 ~ IMU5
    for imu in range(1, 6):
        for feat in per_imu_feats:
            cols.append(f"IMU{imu}_{feat}")

    # 상대 feature 10채널
    rel_feats = [
        "REL_LH_pitch",
        "REL_LH_roll",
        "REL_RH_pitch",
        "REL_RH_roll",
        "REL_LK_pitch",
        "REL_LK_roll",
        "REL_RK_pitch",
        "REL_RK_roll",
        "REL_THIGH_yaw_diff",
        "REL_ANKLE_yaw_diff",
    ]
    cols.extend(rel_feats)

    assert len(cols) == 1 + 90, f"컬럼 개수가 91이 아니에요: {len(cols)}"
    return cols


def main():
    # 파일 선택 창
    root = tk.Tk()
    root.withdraw()

    file_path = filedialog.askopenfilename(
        title="TeraTerm 90채널 로그(txt/csv) 선택",
        filetypes=[
            ("Text / CSV files", "*.txt *.csv"),
            ("All files", "*.*"),
        ]
    )

    if not file_path:
        print("❌ 파일을 선택하지 않았습니다.")
        return

    print("[INFO] 선택한 파일:", file_path)

    # ✅ 확장자는 .txt 여도, 안에가 콤마(,)로 구분돼 있으니까 read_csv 그대로 사용
    df = pd.read_csv(file_path, header=0)
    print("[INFO] 원본 컬럼:", df.columns.tolist())
    print("[INFO] 원본 shape:", df.shape)

    new_cols = build_90ch_column_names()

    if len(df.columns) != len(new_cols):
        raise ValueError(
            f"컬럼 수 불일치: 원본 {len(df.columns)}개 vs 새 이름 {len(new_cols)}개"
        )

    # 컬럼명 교체 (t,ch00,... → Timestep, IMU1_ax, ... REL_ANKLE_yaw_diff)
    df.columns = new_cols

    # 타입 정리 (선택 사항)
    # Timestep을 정수로 바꾸고, 나머지는 float로
    try:
        df["Timestep"] = df["Timestep"].astype(float).astype(int)
    except Exception:
        pass
    
    import re

    for c in df.columns:
    # 1) 앞뒤 공백 제거
        df[c] = df[c].astype(str).str.strip()

    # 2) 숫자 사이 텅 빈 공백 제거
        df[c] = df[c].apply(lambda x: re.sub(r'\s+', '', x))

    # 3) float 변환
        df[c] = df[c].astype(float)



    base, ext = os.path.splitext(file_path)
    out_path = base + "_named.csv"
    df.to_csv(out_path, index=False, encoding="utf-8-sig")

    print("[INFO] 변환 완료 →", out_path)
    print("[INFO] 변환 후 컬럼 예시:", df.columns[:20].tolist(), "...")


if __name__ == "__main__":
    main()
