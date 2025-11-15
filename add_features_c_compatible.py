import numpy as np
import pandas as pd
import tkinter as tk
from tkinter import filedialog
import os

def add_c_compatible_features(df: pd.DataFrame, fs: float = 50.0):
    """
    df: IMU 로그 전체 (여러 move 포함 가능)
        컬럼 예: Timestep, IMU1_ax..IMU1_gz, ..., IMU5_gx, IMU5_gz, move, label
    fs: 샘플링 주파수(Hz) - 지민님은 50Hz
    반환: 각 IMU별로 a_mag, g_mag, pitch, roll, jerk_a_mag, yaw_int 컬럼이 추가된 DataFrame
    """
    dt = 1.0 / fs

    # 몇 개 IMU 있는지 자동 탐지 (IMU1_ax, IMU2_ax ... 를 보고 판단)
    imu_ids = []
    for k in range(1, 10):
        if f"IMU{k}_ax" in df.columns:
            imu_ids.append(k)
        else:
            break

    if not imu_ids:
        raise ValueError("IMU*_ax 형식의 컬럼을 찾지 못했습니다. 컬럼명을 확인해 주세요.")

    has_move = "move" in df.columns

    for k in imu_ids:
        # float32로 통일 (STM32와 최대한 비슷하게)
        ax = df[f"IMU{k}_ax"].astype(np.float32).to_numpy()
        ay = df[f"IMU{k}_ay"].astype(np.float32).to_numpy()
        az = df[f"IMU{k}_az"].astype(np.float32).to_numpy()
        gx = df[f"IMU{k}_gx"].astype(np.float32).to_numpy()
        gy = df[f"IMU{k}_gy"].astype(np.float32).to_numpy()
        gz = df[f"IMU{k}_gz"].astype(np.float32).to_numpy()

        # 1) magnitude
        a_mag = np.sqrt(ax*ax + ay*ay + az*az, dtype=np.float32)
        g_mag = np.sqrt(gx*gx + gy*gy + gz*gz, dtype=np.float32)
        df[f"IMU{k}_a_mag"] = a_mag
        df[f"IMU{k}_g_mag"] = g_mag

        # 2) pitch / roll (rad 단위)
        pitch = np.arctan2(-ax, np.sqrt(ay*ay + az*az + 1e-12, dtype=np.float32)).astype(np.float32)
        roll  = np.arctan2(ay, az + 1e-12).astype(np.float32)
        df[f"IMU{k}_pitch"] = pitch
        df[f"IMU{k}_roll"]  = roll

        # 3) jerk(a_mag): move별로 t=0에서 0으로 초기화
        jerk = np.zeros_like(a_mag, dtype=np.float32)
        if has_move:
            for mv, idx in df.groupby("move").groups.items():
                idx = np.array(sorted(list(idx)))
                am = a_mag[idx]
                # t=0: 0, t>0: (a[t]-a[t-1]) * fs
                j_seg = np.zeros_like(am, dtype=np.float32)
                if len(am) > 1:
                    j_seg[1:] = (am[1:] - am[:-1]) * fs
                jerk[idx] = j_seg
        else:
            if len(a_mag) > 1:
                jerk[1:] = (a_mag[1:] - a_mag[:-1]) * fs
        df[f"IMU{k}_jerk_a_mag"] = jerk

        # 4) yaw_int: move마다 0에서 시작, yaw[t] = yaw[t-1] + gz[t]*dt
        yaw = np.zeros_like(gz, dtype=np.float32)
        if has_move:
            for mv, idx in df.groupby("move").groups.items():
                idx = np.array(sorted(list(idx)))
                gz_seg = gz[idx]
                y_seg = np.zeros_like(gz_seg, dtype=np.float32)
                for i in range(1, len(gz_seg)):
                    y_seg[i] = y_seg[i-1] + gz_seg[i] * dt
                yaw[idx] = y_seg
        else:
            y_seg = np.zeros_like(gz, dtype=np.float32)
            for i in range(1, len(gz)):
                y_seg[i] = y_seg[i-1] + gz[i] * dt
            yaw = y_seg
        df[f"IMU{k}_yaw_int"] = yaw

    return df


def main():
    # 파일 선택 창 열기
    root = tk.Tk()
    root.withdraw()

    file_path = filedialog.askopenfilename(
        title="IMU CSV 파일 선택",
        filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
    )

    if not file_path:
        print("파일을 선택하지 않았습니다.")
        return

    print("[INFO] 선택한 파일:", file_path)

    # CSV 로드
    df = pd.read_csv(file_path)
    print("[INFO] 원본 컬럼 수:", len(df.columns), "| 원본 행 수:", len(df))

    # feature 추가
    FS = 50.0  # Hz (지민 프로젝트 기준)
    df_feat = add_c_compatible_features(df, fs=FS)

    print("[INFO] feature 추가 후 컬럼 수:", len(df_feat.columns))

    # 저장 경로
    base, ext = os.path.splitext(file_path)
    out_path = base + "_feat.csv"

    df_feat.to_csv(out_path, index=False, encoding="utf-8-sig")
    print("✅ 완료: feature 포함 CSV 저장됨")
    print("   →", out_path)


if __name__ == "__main__":
    main()
