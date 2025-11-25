import numpy as np
import pandas as pd
import tkinter as tk
from tkinter import filedialog
import os

# === C 코드와 맞추기 위한 상수들 (MPU-9250, ±500dps 기준) ===
GYRO_LSB_PER_DPS = np.float32(65.5)                      # C: GYRO_LSB_PER_DPS
GYRO_SCALE_RAD   = np.float32(np.pi / 180.0 / 65.5)      # C: GYRO_SCALE_RAD
QUAT_ALPHA       = np.float32(0.02)                      # C: QUAT_ALPHA
IMU_EPS          = np.float32(1e-12)


# === C 코드와 동일한 쿼터니언 연산 함수들 ===
def quat_normalize_np(q: np.ndarray) -> np.ndarray:
    """q: shape (4,)"""
    n = np.sqrt(np.sum(q * q, dtype=np.float32))
    if n < 1e-6:
        return q.astype(np.float32)
    return (q / n).astype(np.float32)


def quat_mul_np(q: np.ndarray, r: np.ndarray) -> np.ndarray:
    """
    C 코드의 quat_mul(q, r, out)와 동일한 Hamilton 곱
    q, r: shape (4,) = [w, x, y, z]
    """
    w1, x1, y1, z1 = q
    w2, x2, y2, z2 = r

    out_w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    out_x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    out_y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    out_z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return np.array([out_w, out_x, out_y, out_z], dtype=np.float32)


def euler_to_quat_np(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    C의 euler_to_quat(roll, pitch, yaw, q)와 동일한 수식.
    roll, pitch, yaw: rad
    """
    roll  = np.float32(roll)
    pitch = np.float32(pitch)
    yaw   = np.float32(yaw)

    cr = np.cos(roll * 0.5, dtype=np.float32)
    sr = np.sin(roll * 0.5, dtype=np.float32)
    cp = np.cos(pitch * 0.5, dtype=np.float32)
    sp = np.sin(pitch * 0.5, dtype=np.float32)
    cy = np.cos(yaw * 0.5, dtype=np.float32)
    sy = np.sin(yaw * 0.5, dtype=np.float32)

    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy

    return np.array([qw, qx, qy, qz], dtype=np.float32)


def add_c_compatible_features(df: pd.DataFrame, fs: float = 50.0):
    """
    df: IMU 로그 전체 (여러 move 포함 가능)
        컬럼 예: Timestep, IMU1_ax..IMU1_gz, ..., IMU5_gx, IMU5_gz, move, label
    fs: 샘플링 주파수(Hz) - 50Hz
    반환: 각 IMU별로
          a_mag, g_mag, pitch, roll, jerk_a_mag, yaw_int
          + quaternion(qw, qx, qy, qz)
          + 상대 feature 10채널(REL_*) 컬럼이 추가된 DataFrame
    """
    dt = np.float32(1.0 / fs)

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

    # --------- 1) 각 IMU별 단일 feature + yaw_int + quaternion ----------
    for k in imu_ids:
        # float32로 통일
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

        # 2) pitch / roll (rad 단위) - C 코드와 동일한 수식
        pitch = np.arctan2(-ax, np.sqrt(ay*ay + az*az + IMU_EPS, dtype=np.float32)).astype(np.float32)
        roll  = np.arctan2(ay, az + IMU_EPS).astype(np.float32)
        df[f"IMU{k}_pitch"] = pitch
        df[f"IMU{k}_roll"]  = roll

        # 3) jerk(a_mag): move별로 t=0에서 0으로 초기화
        jerk = np.zeros_like(a_mag, dtype=np.float32)
        if has_move:
            for mv, idx in df.groupby("move").groups.items():
                idx = np.array(sorted(list(idx)))
                am = a_mag[idx]
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

        # 5) Quaternion(qw, qx, qy, qz) 추가
        #    C 코드의 ImuQuatState + compute_features_for_frame() 와 동일한 방식
        N = len(df)
        qw = np.zeros(N, dtype=np.float32)
        qx = np.zeros(N, dtype=np.float32)
        qy = np.zeros(N, dtype=np.float32)
        qz = np.zeros(N, dtype=np.float32)

        if has_move:
            # move마다 quaternion 상태를 0에서 다시 시작
            for mv, idx in df.groupby("move").groups.items():
                idx_sorted = np.array(sorted(list(idx)))
                if len(idx_sorted) == 0:
                    continue

                q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
                init = False

                for pos in idx_sorted:
                    r = float(roll[pos])
                    p = float(pitch[pos])
                    gx_lsb = float(gx[pos])
                    gy_lsb = float(gy[pos])
                    gz_lsb = float(gz[pos])

                    if not init:
                        # 초기 샘플: accel 기반 roll/pitch, yaw=0
                        q = euler_to_quat_np(r, p, 0.0)
                        q = quat_normalize_np(q)
                        init = True
                    else:
                        # --- 자이로 LSB → rad/s ---
                        wx = np.float32(gx_lsb) * GYRO_SCALE_RAD
                        wy = np.float32(gy_lsb) * GYRO_SCALE_RAD
                        wz = np.float32(gz_lsb) * GYRO_SCALE_RAD

                        w_q = np.array([0.0, wx, wy, wz], dtype=np.float32)

                        # dq/dt = 0.5 * q ⊗ [0, wx, wy, wz]
                        dq = quat_mul_np(q, w_q)

                        q_gyro = q + 0.5 * dt * dq
                        q_gyro = quat_normalize_np(q_gyro)

                        # accel 기반 quaternion (yaw=0)
                        q_acc = euler_to_quat_np(r, p, 0.0)
                        q_acc = quat_normalize_np(q_acc)

                        # complementary filter
                        q = (1.0 - QUAT_ALPHA) * q_gyro + QUAT_ALPHA * q_acc
                        q = quat_normalize_np(q)

                    qw[pos], qx[pos], qy[pos], qz[pos] = q

        else:
            # 전체를 한 세그먼트로 보고 quaternion 상태 1번만 초기화
            q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
            init = False

            for pos in range(N):
                r = float(roll[pos])
                p = float(pitch[pos])
                gx_lsb = float(gx[pos])
                gy_lsb = float(gy[pos])
                gz_lsb = float(gz[pos])

                if not init:
                    q = euler_to_quat_np(r, p, 0.0)
                    q = quat_normalize_np(q)
                    init = True
                else:
                    wx = np.float32(gx_lsb) * GYRO_SCALE_RAD
                    wy = np.float32(gy_lsb) * GYRO_SCALE_RAD
                    wz = np.float32(gz_lsb) * GYRO_SCALE_RAD

                    w_q = np.array([0.0, wx, wy, wz], dtype=np.float32)
                    dq = quat_mul_np(q, w_q)

                    q_gyro = q + 0.5 * dt * dq
                    q_gyro = quat_normalize_np(q_gyro)

                    q_acc = euler_to_quat_np(r, p, 0.0)
                    q_acc = quat_normalize_np(q_acc)

                    q = (1.0 - QUAT_ALPHA) * q_gyro + QUAT_ALPHA * q_acc
                    q = quat_normalize_np(q)

                qw[pos], qx[pos], qy[pos], qz[pos] = q

        # DataFrame에 저장 (IMU별 4채널)
        df[f"IMU{k}_qw"] = qw
        df[f"IMU{k}_qx"] = qx
        df[f"IMU{k}_qy"] = qy
        df[f"IMU{k}_qz"] = qz

    # --------- 2) IMU 간 상대 feature (pitch/roll/yaw_int 차이) ----------
    # IMU1~5, pitch/roll/yaw_int가 모두 존재할 때만 계산
    needed_cols = [
        "IMU1_pitch", "IMU2_pitch", "IMU3_pitch", "IMU4_pitch", "IMU5_pitch",
        "IMU1_roll",  "IMU2_roll",  "IMU3_roll",  "IMU4_roll",  "IMU5_roll",
        "IMU2_yaw_int", "IMU3_yaw_int", "IMU4_yaw_int", "IMU5_yaw_int",
    ]
    if all(col in df.columns for col in needed_cols):
        # Left Hip (허리-왼 허벅지) : IMU2 - IMU1
        df["REL_LH_pitch"] = df["IMU2_pitch"] - df["IMU1_pitch"]
        df["REL_LH_roll"]  = df["IMU2_roll"]  - df["IMU1_roll"]

        # Right Hip (허리-오른 허벅지) : IMU3 - IMU1
        df["REL_RH_pitch"] = df["IMU3_pitch"] - df["IMU1_pitch"]
        df["REL_RH_roll"]  = df["IMU3_roll"]  - df["IMU1_roll"]

        # Left Knee (왼 허벅지-왼 발목) : IMU4 - IMU2
        df["REL_LK_pitch"] = df["IMU4_pitch"] - df["IMU2_pitch"]
        df["REL_LK_roll"]  = df["IMU4_roll"]  - df["IMU2_roll"]

        # Right Knee (오른 허벅지-오른 발목) : IMU5 - IMU3
        df["REL_RK_pitch"] = df["IMU5_pitch"] - df["IMU3_pitch"]
        df["REL_RK_roll"]  = df["IMU5_roll"]  - df["IMU3_roll"]

        # Thigh yaw diff (왼/오른 허벅지 비틀림) : IMU2 - IMU3
        df["REL_THIGH_yaw_diff"] = df["IMU2_yaw_int"] - df["IMU3_yaw_int"]

        # Ankle yaw diff (왼/오른 발목 비틀림) : IMU4 - IMU5
        df["REL_ANKLE_yaw_diff"] = df["IMU4_yaw_int"] - df["IMU5_yaw_int"]

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

    # feature + quaternion + 상대 feature 추가
    FS = 50.0  # Hz
    df_feat = add_c_compatible_features(df, fs=FS)

    print("[INFO] feature + quaternion + REL 추가 후 컬럼 수:", len(df_feat.columns))

    # 저장 경로
    base, ext = os.path.splitext(file_path)
    out_path = base + "_feat_quat_rel.csv"

    df_feat.to_csv(out_path, index=False, encoding="utf-8-sig")
    print("✅ 완료: feature + quaternion + 상대 feature 포함 CSV 저장됨")
    print("   →", out_path)


if __name__ == "__main__":
    main()
