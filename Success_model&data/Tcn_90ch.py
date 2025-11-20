# ============================================
# 5-Class TCN — Cube.AI-friendly (90채널: RAW30 + FEAT50 + REL10) + Grid Search
# - 입력: (L=128, C=90)
#   = RAW30
#     + 5 IMUs × (a_mag,g_mag,pitch,roll,jerk_a_mag,yaw_int,qw,qx,qy,qz) = 50
#     + REL 10 (IMU 간 pitch/roll/yaw 차이)
# - 전처리: 선형 리샘플 → 채널별 z-score(μ/σ 저장)
# - 모델: Conv1D+BN+ReLU + Residual TCN blocks → GAP1D → Dense(softmax, 5)
# - 그리드 서치: ch, k, p, use_dilation 조합 탐색
# - 출력:
#   - final_export_cubeai_90ch_best.h5
#   - tcn_norm_stats_90ch.json (라벨 정보 포함)
# ============================================

from sklearn.utils.class_weight import compute_class_weight
from sklearn.metrics import confusion_matrix, classification_report

import os, json
import numpy as np
import pandas as pd
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers

# ---------- 경로/설정 ----------
BASE = r"C:\Users\hjl99\Desktop\IMU_MODEL_DATA"

# ★ CSV: raw IMU + move + label 이 들어있는 파일이면 됨
#   (예전 *_feat.csv 써도 되는데, 아래 코드가 RAW에서 다시 FEAT/REL을 계산하니까
#    기존 feat 컬럼은 무시됨)
CSV_PATHS = [
    f"{BASE}/imu_label_0_ALL_feat_quat_rel.csv",
    f"{BASE}/imu_label_1_ALL_feat_quat_rel.csv",
    f"{BASE}/imu_label_2_ALL_feat_quat_rel.csv",
    f"{BASE}/imu_label_3_ALL_feat_quat_rel.csv",
    f"{BASE}/imu_label_4_ALL_feat_quat_rel.csv",
]

# ★ 보드 TCN_L과 반드시 같게 맞춰야 함
L_TARGET = 256
NUM_CLASSES = 5  # 5 클래스

# ---------- C 코드와 동일한 상수들 ----------
IMU_FS   = 50.0
IMU_DT   = 1.0 / IMU_FS
IMU_EPS  = 1e-12

NUM_IMU        = 5
RAW_CH_PER_IMU = 6   # ax, ay, az, gx, gy, gz

FEAT_SCALAR_PER_IMU = 6   # a_mag, g_mag, pitch, roll, jerk_a_mag, yaw_int
FEAT_QUAT_PER_IMU   = 4   # qw, qx, qy, qz
FEAT_PER_IMU        = FEAT_SCALAR_PER_IMU + FEAT_QUAT_PER_IMU  # 10
FEAT_CHANNELS       = NUM_IMU * FEAT_PER_IMU                   # 50

REL_CHANNELS = 10

GYRO_LSB_PER_DPS = 65.5
GYRO_SCALE_RAD   = np.pi / 180.0 / GYRO_LSB_PER_DPS
QUAT_ALPHA       = 0.02

# ---------- 채널 이름 정의 (보드 tmp_ext 순서와 1:1) ----------

IMU_IDS   = list(range(1, 6))
RAW_AXES  = ["ax", "ay", "az", "gx", "gy", "gz"]

# RAW 30
RAW_CHANNELS = [
    f"IMU{k}_{a}" for k in IMU_IDS for a in RAW_AXES
]  # 5×6 = 30

# per-IMU scalar 6
FEAT_SCALAR_AXES = ["a_mag", "g_mag", "pitch", "roll", "jerk_a_mag", "yaw_int"]
FEAT_SCALAR_CHANNELS = [
    f"IMU{k}_{a}" for k in IMU_IDS for a in FEAT_SCALAR_AXES
]  # 5×6 = 30

# per-IMU quat 4
FEAT_QUAT_AXES = ["qw", "qx", "qy", "qz"]
FEAT_QUAT_CHANNELS = [
    f"IMU{k}_{a}" for k in IMU_IDS for a in FEAT_QUAT_AXES
]  # 5×4 = 20

# REL 10 (C 코드 순서 그대로)
REL_CHANNEL_NAMES = [
    "REL_LH_pitch",   # 0
    "REL_LH_roll",    # 1
    "REL_RH_pitch",   # 2
    "REL_RH_roll",    # 3
    "REL_LK_pitch",   # 4
    "REL_LK_roll",    # 5
    "REL_RK_pitch",   # 6
    "REL_RK_roll",    # 7
    "REL_THIGH_yaw",  # 8
    "REL_ANKLE_yaw",  # 9
]

# 최종 90채널 이름 (tmp_ext와 완전히 동일한 순서)
CHANNELS_90 = (
    RAW_CHANNELS
    + FEAT_SCALAR_CHANNELS
    + FEAT_QUAT_CHANNELS
    + REL_CHANNEL_NAMES
)
assert len(CHANNELS_90) == 90

# ---------- 유틸: C와 동일한 쿼터니언/피쳐 계산 ----------

def quat_normalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    if n < 1e-6:
        return q
    return q / n


def quat_mul(q: np.ndarray, r: np.ndarray) -> np.ndarray:
    """
    C 코드 quat_mul_c와 동일:
    out = q ⊗ r
    """
    w1, x1, y1, z1 = q
    w2, x2, y2, z2 = r
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=np.float32,
    )


def euler_to_quat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    C 코드 euler_to_quat_c(roll, pitch, yaw) 그대로
    """
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return np.array([w, x, y, z], dtype=np.float32)


def compute_features_for_move(raw_Tx30: np.ndarray) -> np.ndarray:
    """
    raw_Tx30: [T, 30] = IMU1..5 × (ax,ay,az,gx,gy,gz)
    C 코드 compute_features_for_frame() + REL 계산을
    "한 move 단위"로 그대로 옮긴 버전.
    반환: [T, 90] = RAW30 + FEAT50 + REL10
    """

    T = raw_Tx30.shape[0]
    raw_Tx30 = raw_Tx30.astype(np.float32)

    # [T, 5, 6] 로 reshape (IMU 순서, 축 순서 C와 동일)
    raw_imu = raw_Tx30.reshape(T, NUM_IMU, RAW_CH_PER_IMU)

    feat_Tx50 = np.zeros((T, FEAT_CHANNELS), dtype=np.float32)
    rel_Tx10  = np.zeros((T, REL_CHANNELS), dtype=np.float32)

    # C의 g_feat_state, g_quat_state에 해당하는 상태들
    prev_a_mag     = np.zeros(NUM_IMU, dtype=np.float32)
    yaw_int        = np.zeros(NUM_IMU, dtype=np.float32)
    first_sample   = np.ones(NUM_IMU, dtype=bool)

    quat           = np.zeros((NUM_IMU, 4), dtype=np.float32)
    quat[:, 0]     = 1.0  # w=1, x=y=z=0
    quat_init      = np.zeros(NUM_IMU, dtype=bool)

    for t in range(T):
        pitch_arr = np.zeros(NUM_IMU, dtype=np.float32)
        roll_arr  = np.zeros(NUM_IMU, dtype=np.float32)
        yaw_arr   = np.zeros(NUM_IMU, dtype=np.float32)

        for imu in range(NUM_IMU):
            ax = raw_imu[t, imu, 0]
            ay = raw_imu[t, imu, 1]
            az = raw_imu[t, imu, 2]
            gx = raw_imu[t, imu, 3]
            gy = raw_imu[t, imu, 4]
            gz = raw_imu[t, imu, 5]

            # 1) a_mag, g_mag
            a_mag = np.sqrt(ax * ax + ay * ay + az * az)
            g_mag = np.sqrt(gx * gx + gy * gy + gz * gz)

            # 2) pitch, roll (C와 동일 수식)
            denom = np.sqrt(ay * ay + az * az + IMU_EPS)
            pitch = np.arctan2(-ax, denom)
            roll  = np.arctan2(ay, az + IMU_EPS)

            # 3) jerk(|a|) : (a[t]-a[t-1]) * fs
            if first_sample[imu]:
                jerk = 0.0
                first_sample[imu] = False
            else:
                jerk = (a_mag - prev_a_mag[imu]) * IMU_FS
            prev_a_mag[imu] = a_mag

            # 4) yaw_int : gz(LBS) * dt 누적
            yaw_int[imu] += gz * IMU_DT
            yaw_val = yaw_int[imu]

            # 5) quaternion 보정 (C 코드와 동일 흐름)
            if not quat_init[imu]:
                q = euler_to_quat(roll, pitch, 0.0)
                q = quat_normalize(q)
                quat[imu] = q
                quat_init[imu] = True
            else:
                # gyro LSB → rad/s
                wx = gx * GYRO_SCALE_RAD
                wy = gy * GYRO_SCALE_RAD
                wz = gz * GYRO_SCALE_RAD

                w_q = np.array([0.0, wx, wy, wz], dtype=np.float32)
                dq  = quat_mul(quat[imu], w_q)
                q_g = quat[imu] + 0.5 * IMU_DT * dq
                q_g = quat_normalize(q_g)

                q_acc = euler_to_quat(roll, pitch, 0.0)
                q_acc = quat_normalize(q_acc)

                q_new = (1.0 - QUAT_ALPHA) * q_g + QUAT_ALPHA * q_acc
                quat[imu] = quat_normalize(q_new)

            # 6) feat_buffer에 저장 (IMU별 10채널)
            fbase = imu * FEAT_PER_IMU
            feat_Tx50[t, fbase + 0] = a_mag
            feat_Tx50[t, fbase + 1] = g_mag
            feat_Tx50[t, fbase + 2] = pitch
            feat_Tx50[t, fbase + 3] = roll
            feat_Tx50[t, fbase + 4] = jerk
            feat_Tx50[t, fbase + 5] = yaw_val
            feat_Tx50[t, fbase + 6] = quat[imu, 0]  # qw
            feat_Tx50[t, fbase + 7] = quat[imu, 1]  # qx
            feat_Tx50[t, fbase + 8] = quat[imu, 2]  # qy
            feat_Tx50[t, fbase + 9] = quat[imu, 3]  # qz

            pitch_arr[imu] = pitch
            roll_arr[imu]  = roll
            yaw_arr[imu]   = yaw_val

        # 7) REL feature 10개 (C 코드와 동일)
        rel_Tx10[t, 0] = pitch_arr[1] - pitch_arr[0]   # REL_LH_pitch
        rel_Tx10[t, 1] = roll_arr[1]  - roll_arr[0]    # REL_LH_roll

        rel_Tx10[t, 2] = pitch_arr[2] - pitch_arr[0]   # REL_RH_pitch
        rel_Tx10[t, 3] = roll_arr[2]  - roll_arr[0]    # REL_RH_roll

        rel_Tx10[t, 4] = pitch_arr[3] - pitch_arr[1]   # REL_LK_pitch
        rel_Tx10[t, 5] = roll_arr[3]  - roll_arr[1]    # REL_LK_roll

        rel_Tx10[t, 6] = pitch_arr[4] - pitch_arr[2]   # REL_RK_pitch
        rel_Tx10[t, 7] = roll_arr[4]  - roll_arr[2]    # REL_RK_roll

        rel_Tx10[t, 8] = yaw_arr[1]   - yaw_arr[2]     # REL_THIGH_yaw
        rel_Tx10[t, 9] = yaw_arr[3]   - yaw_arr[4]     # REL_ANKLE_yaw

    # 최종: RAW30 + FEAT50 + REL10
    return np.concatenate([raw_Tx30, feat_Tx50, rel_Tx10], axis=1)


# ---------- 유틸: CSV에서 RAW + move/label 뽑기 ----------

def _pick_raw_move_label(df: pd.DataFrame) -> pd.DataFrame:
    """
    - *_tick, 기타 필요없는 컬럼은 무시
    - RAW 30채널 + move + label만 남기고 이름을 표준화
    """
    cols = list(df.columns)
    low2orig = {c.lower(): c for c in cols}

    # Timestep 있으면 타임 순서 정렬
    t_col = low2orig.get("timestep", None)
    if t_col is not None:
        df = df.sort_values(t_col).reset_index(drop=True)

    # move/label 컬럼 찾기 (대소문자 무시)
    move_col = low2orig.get("move", None)
    label_col = low2orig.get("label", None)
    if move_col is None or label_col is None:
        raise ValueError("move/label 컬럼이 필요합니다.")

    # RAW 채널 컬럼 매핑
    raw_cols_real = []
    for name in RAW_CHANNELS:
        key = name.lower()
        if key not in low2orig:
            raise ValueError(f"필수 RAW 컬럼 누락: {name}")
        raw_cols_real.append(low2orig[key])

    df2 = df[raw_cols_real + [move_col, label_col]].copy()

    # 표준 이름으로 rename
    rename_map = {old: new for old, new in zip(raw_cols_real, RAW_CHANNELS)}
    rename_map[move_col] = "move"
    rename_map[label_col] = "label"
    df2.rename(columns=rename_map, inplace=True)

    return df2


def _resample_to_L(x_T_C: np.ndarray, L: int = 128) -> np.ndarray:
    """
    x: [T, C] → 선형 보간으로 [L, C]
    (보드 C 코드 tcn_linear_resample와 동일한 방식)
    """
    T, C = x_T_C.shape
    if T < 2:
        return np.repeat(x_T_C.astype(np.float32), L, axis=0)

    x_T_C = x_T_C.astype(np.float32)
    t_src = np.linspace(0.0, 1.0, num=T, dtype=np.float32)
    t_dst = np.linspace(0.0, 1.0, num=L, dtype=np.float32)
    out = np.empty((L, C), dtype=np.float32)

    for c in range(C):
        out[:, c] = np.interp(t_dst, t_src, x_T_C[:, c])

    return out


def build_dataset(paths):
    X_list, y_list = [], []
    for p in paths:
        if not os.path.exists(p):
            print(f"[WARN] not found: {p}")
            continue

        df = pd.read_csv(p)
        df = _pick_raw_move_label(df)  # RAW 30 + move + label

        # move 단위로 한 동작 = 한 샘플
        for mv, g in df.groupby("move"):
            raw = g[RAW_CHANNELS].astype(np.float32).values  # [T, 30]

            # C 코드와 동일한 방식으로 90채널 feature 계산
            feat90 = compute_features_for_move(raw)          # [T, 90]

            # L_TARGET으로 리샘플
            x = _resample_to_L(feat90, L=L_TARGET)           # [L_TARGET, 90]
            X_list.append(x)

            # 무브 내 라벨 혼재 시 다수결(평균 반올림)
            y_list.append(int(np.round(g["label"].astype(int).mean())))

    X = np.stack(X_list, axis=0)          # [N, L_TARGET, 90]
    y = np.array(y_list, dtype=np.int32)  # [N]
    return X, y


# ---------- 데이터 로드 ----------
X_all, y_all = build_dataset(CSV_PATHS)
print("Original dataset:", X_all.shape, "class dist:", np.bincount(y_all))

N, L_used, C = X_all.shape
assert L_used == L_TARGET, f"L mismatch: {L_used} vs {L_TARGET}"
assert C == len(CHANNELS_90) == 90, f"C mismatch: {C}"

# ---------- train / val / test split ----------
rng = np.random.RandomState(42)
idx = np.arange(N)
rng.shuffle(idx)

tr_end = int(N * 0.6)   # 60% train
va_end = int(N * 0.8)   # 20% val
tr_idx = idx[:tr_end]
va_idx = idx[tr_end:va_end]
te_idx = idx[va_end:]

Xtr_raw = X_all[tr_idx]
Xva_raw = X_all[va_idx]
Xte_raw = X_all[te_idx]

ytr = y_all[tr_idx]
yva = y_all[va_idx]
yte = y_all[te_idx]

print("Train shape:", Xtr_raw.shape, "class dist:", np.bincount(ytr))
print("Val   shape:", Xva_raw.shape, "class dist:", np.bincount(yva))
print("Test  shape:", Xte_raw.shape, "class dist:", np.bincount(yte))

# ---------- 정규화 (train 기준 μ/σ, 채널별) ----------
flat_tr = Xtr_raw.reshape(-1, Xtr_raw.shape[-1])  # [Ntr*L, 90]
mu = flat_tr.mean(axis=0)
sigma = flat_tr.std(axis=0)
sigma[sigma < 1e-6] = 1.0   # 0 division 방지

Xtr = (Xtr_raw - mu) / sigma
Xva = (Xva_raw - mu) / sigma
Xte = (Xte_raw - mu) / sigma

# 디버깅용 전체 정규화 버전
X_all_norm = (X_all.astype(np.float32) - mu) / sigma
# ---------- 🔧 Train set 오버샘플링으로 클래스 균형 맞추기 ----------
from collections import Counter

print("\n[INFO] Before balancing, train label dist:", Counter(ytr))

unique, counts = np.unique(ytr, return_counts=True)
max_count = counts.max()  # 가장 많은 클래스 개수에 맞춰서 나머지 클래스를 복제

rng_bal = np.random.RandomState(2025)

idx_balanced_list = []
for cls, cnt in zip(unique, counts):
    cls_idx = np.where(ytr == cls)[0]  # 이 라벨에 해당하는 인덱스들
    if cnt == 0:
        continue

    if cnt < max_count:
        # 부족한 만큼 랜덤 복제 (with replacement)
        extra_idx = rng_bal.choice(cls_idx, size=max_count - cnt, replace=True)
        new_idx = np.concatenate([cls_idx, extra_idx])
    else:
        # 이미 max_count 이상이면 그대로
        new_idx = cls_idx

    idx_balanced_list.append(new_idx)

# 모든 클래스를 이어 붙여서 섞기
idx_balanced = np.concatenate(idx_balanced_list)
rng_bal.shuffle(idx_balanced)

Xtr_bal = Xtr[idx_balanced]
ytr_bal = ytr[idx_balanced]

print("[INFO] After  balancing, train label dist:", Counter(ytr_bal))
print("[INFO] Xtr_bal shape:", Xtr_bal.shape)
# ---------- 🔍 디버그: 각 라벨별로 무브 5개씩 CSV로 저장 (모델 입력 직전) ----------
dbg_dir = f"{BASE}/dbg_inputs_per_label_90ch"
os.makedirs(dbg_dir, exist_ok=True)

max_per_label = 5  # 라벨당 최대 몇 개 저장할지

print(f"\n[DEBUG] 라벨별로 최대 {max_per_label}개씩, 모델 입력 기준(정규화 후) CSV로 저장합니다.")
for cls in range(NUM_CLASSES):   # 0,1,2,3,4
    idx_cls = np.where(y_all == cls)[0]
    if len(idx_cls) == 0:
        print(f"[WARN] label {cls} 샘플이 없습니다.")
        continue

    # 라벨별로 샘플 인덱스 섞고 최대 5개 선택
    rng = np.random.RandomState(1234 + cls)
    rng.shuffle(idx_cls)
    sel_idx = idx_cls[:max_per_label]

    for j, idx0 in enumerate(sel_idx):
        # X_all_norm: (N, L_TARGET, 90) = 모델 입력 직전 정규화된 데이터
        x_one = X_all_norm[idx0]   # (L_TARGET, 90)

        # Timestep + 90채널 + move + label 형식으로 CSV 만들기
        df_dbg = pd.DataFrame(x_one, columns=CHANNELS_90)
        df_dbg.insert(0, "Timestep", np.arange(L_TARGET, dtype=np.int32))

        # 여기 move 값은 "이 CSV 안에서의 샘플 번호" 정도라 의미용이니,
        # 필요하면 나중에 실제 move ID랑 매핑하는 로직을 추가해도 됨
        df_dbg["move"] = np.ones(L_TARGET, dtype=np.int32) * (j + 1)
        df_dbg["label"] = np.ones(L_TARGET, dtype=np.int32) * cls

        out_path = f"{dbg_dir}/label{cls}_sample{j}_idx{idx0}.csv"
        df_dbg.to_csv(out_path, index=False, encoding="utf-8-sig")
        print(f"[DEBUG] saved: {out_path}")
# ---------- μ/σ + 라벨 정보 JSON 저장 (C 코드에서 사용) ----------
stats_path = f"{BASE}/tcn_norm_stats_90ch.json"
norm_stats = {
    "L": int(L_TARGET),
    "channels": CHANNELS_90,            # 길이 90
    "mu": mu.tolist(),                  # 90개
    "sigma": sigma.tolist(),            # 90개
    "label_map": {str(i): int(i) for i in range(NUM_CLASSES)},
}
with open(stats_path, "w", encoding="utf-8") as f:
    json.dump(norm_stats, f, indent=2, ensure_ascii=False)

print("Saved norm stats to:", stats_path)

# ---------- 클래스 불균형 보정 (class_weight) ----------
classes = np.unique(ytr)
class_weights_arr = compute_class_weight(
    class_weight="balanced",
    classes=classes,
    y=ytr
)
class_weight = {int(c): float(w) for c, w in zip(classes, class_weights_arr)}
print("class_weight:", class_weight)

# ---------- 🔍 디버그: 라벨별로 최대 5개씩 CSV 저장 (MATLAB 시각화용) ----------
dbg_dir = f"{BASE}/dbg_inputs_per_label_90ch"
os.makedirs(dbg_dir, exist_ok=True)

max_per_label = 5  # 라벨당 최대 몇 개 저장할지

for cls in range(NUM_CLASSES):   # 0,1,2,3,4
    idx_cls = np.where(y_all == cls)[0]
    if len(idx_cls) == 0:
        print(f"[WARN] label {cls} 샘플이 없습니다.")
        continue

    rng = np.random.RandomState(1234 + cls)
    rng.shuffle(idx_cls)
    sel_idx = idx_cls[:max_per_label]

    for j, idx0 in enumerate(sel_idx):
        x_one = X_all_norm[idx0]   # (L_TARGET, 90)

        df_dbg = pd.DataFrame(x_one, columns=CHANNELS_90)
        df_dbg.insert(0, "Timestep", np.arange(L_TARGET, dtype=np.int32))
        df_dbg["move"] = np.ones(L_TARGET, dtype=np.int32) * (j + 1)
        df_dbg["label"] = np.ones(L_TARGET, dtype=np.int32) * cls

        out_path = f"{dbg_dir}/label{cls}_sample{j}_idx{idx0}.csv"
        df_dbg.to_csv(out_path, index=False, encoding="utf-8-sig")
        print(f"saved: {out_path}")

# ---------- TCN 모델 (90채널 입력) ----------
def tcn_block(x, ch, k, dilation=1, p=0.3, name_prefix="tcn"):
    h = layers.Conv1D(ch, k, padding="same",
                      dilation_rate=dilation,
                      use_bias=False,
                      name=f"{name_prefix}_conv1_d{dilation}")(x)
    h = layers.BatchNormalization(name=f"{name_prefix}_bn1_d{dilation}")(h)
    h = layers.ReLU(name=f"{name_prefix}_relu1_d{dilation}")(h)
    if p > 0:
        h = layers.Dropout(p, name=f"{name_prefix}_drop1_d{dilation}")(h)

    h = layers.Conv1D(ch, k, padding="same",
                      dilation_rate=dilation,
                      use_bias=False,
                      name=f"{name_prefix}_conv2_d{dilation}")(h)
    h = layers.BatchNormalization(name=f"{name_prefix}_bn2_d{dilation}")(h)

    x = layers.Add(name=f"{name_prefix}_add_d{dilation}")([x, h])
    x = layers.ReLU(name=f"{name_prefix}_out_relu_d{dilation}")(x)
    return x


def tcn_model(input_shape,
              num_classes=5,
              ch=48,
              k=5,
              p=0.2,
              use_dilation=True):
    inp = keras.Input(shape=input_shape, name="imu")  # [L, C]

    # stem
    x = layers.Conv1D(ch, k, padding="same", use_bias=False, name="stem_conv")(inp)
    x = layers.BatchNormalization(name="stem_bn")(x)
    x = layers.ReLU(name="stem_relu")(x)
    if p > 0:
        x = layers.Dropout(p, name="stem_drop")(x)

    # TCN blocks
    d_list = [1, 2, 4] if use_dilation else [1, 1, 1]
    for i, d in enumerate(d_list):
        x = tcn_block(
            x, ch=ch, k=k, dilation=d, p=p,
            name_prefix=f"tcn_block{i+1}"
        )

    # head
    x = layers.GlobalAveragePooling1D(name="gap")(x)
    out = layers.Dense(num_classes, activation="softmax", name="pred")(x)

    return keras.Model(inp, out, name="tcn_squat_5cls_90ch")


# ---------- Grid Search 설정 ----------
GRID_CH = [48, 64]
GRID_K  = [3, 5]
GRID_P  = [0.1, 0.2]
GRID_D  = [True, False]

EPOCHS_PER_TRIAL = 40
BATCH_SIZE = 32

def train_one_config(ch, k, p, use_dilation):
    print(f"\n===== [GRID] ch={ch}, k={k}, p={p}, use_dilation={use_dilation} =====")

    model = tcn_model(
        (L_TARGET, C),
        NUM_CLASSES,
        ch=ch,
        k=k,
        p=p,
        use_dilation=use_dilation
    )

    model.compile(
        optimizer=keras.optimizers.Adam(1e-3),
        loss="sparse_categorical_crossentropy",
        metrics=["accuracy"]
    )

    cbs = [
        keras.callbacks.ReduceLROnPlateau(
            monitor="val_accuracy",
            factor=0.5,
            patience=4,
            verbose=1,
            min_lr=1e-5
        ),
        keras.callbacks.EarlyStopping(
            monitor="val_accuracy",
            patience=8,
            restore_best_weights=True,
            verbose=1
        )
    ]


    history = model.fit(
    Xtr_bal, ytr_bal,                 # ✅ 오버샘플링된 데이터 사용
    validation_data=(Xva, yva),
    epochs=EPOCHS_PER_TRIAL,
    batch_size=BATCH_SIZE,
    verbose=2,
    # class_weight=class_weight,      # ✅ 일단 OFF 권장 (둘 다 쓰면 과하게 튈 수 있음)
    callbacks=cbs
)


    val_acc_hist = history.history["val_accuracy"]
    best_val_acc = float(np.max(val_acc_hist))
    best_epoch = int(np.argmax(val_acc_hist))

    print(f"[GRID] best val_acc={best_val_acc:.4f} at epoch {best_epoch+1}")

    return model, best_val_acc, best_epoch, history


# ---------- 실제 그리드 서치 루프 ----------
results = []
best_model = None
best_score = -1.0
best_cfg = None

trial_id = 0

for ch in GRID_CH:
    for k in GRID_K:
        for p in GRID_P:
            for use_dilation in GRID_D:
                trial_id += 1
                print(f"\n########## Trial {trial_id} ##########")
                print(f"config: ch={ch}, k={k}, p={p}, use_dilation={use_dilation}")

                model, val_acc, best_epoch, history = train_one_config(ch, k, p, use_dilation)

                results.append({
                    "trial": trial_id,
                    "ch": ch,
                    "k": k,
                    "p": p,
                    "use_dilation": use_dilation,
                    "best_val_acc": val_acc,
                    "best_epoch": best_epoch + 1
                })

                if val_acc > best_score:
                    best_score = val_acc
                    best_cfg = {
                        "ch": ch,
                        "k": k,
                        "p": p,
                        "use_dilation": use_dilation
                    }
                    best_model = model
                    print(f"🔺 New BEST config! val_acc={best_score:.4f}, cfg={best_cfg}")

results_df = pd.DataFrame(results).sort_values("best_val_acc", ascending=False)
print("\n===== Grid Search Results (Top) =====")
print(results_df.head(10))

# ---------- 베스트 모델 테스트셋 평가 ----------
if best_model is None:
    raise RuntimeError("그리드 서치에서 best_model이 설정되지 않았습니다.")

test_loss, test_acc = best_model.evaluate(Xte, yte, batch_size=BATCH_SIZE, verbose=0)
print(f"\n✅ BEST CONFIG: {best_cfg}")
print(f"✅ Test accuracy = {test_acc:.4f}, loss = {test_loss:.4f}")
# ---------- 라벨별 성능 확인용: 혼동행렬 & 리포트 ----------
yte_proba = best_model.predict(Xte, batch_size=BATCH_SIZE, verbose=0)
yte_pred  = np.argmax(yte_proba, axis=1)

print("\n[CONFUSION MATRIX - TEST]")
print(confusion_matrix(yte, yte_pred))

print("\n[CLASSIFICATION REPORT - TEST]")
print(classification_report(yte, yte_pred, digits=4))

# ---------- 내보내기 (Cube.AI용 .h5) ----------
save_path = f"{BASE}/final_export_cubeai_90ch_best.h5"
best_model.save(save_path)
print("Saved BEST model to:", save_path)

# ---------- label 4 디버그 (BEST 모델 기준) ----------
idx4 = np.where(y_all == 4)[0]
if len(idx4) > 0:
    X4 = X_all_norm[idx4]

    proba4 = best_model.predict(X4, batch_size=32, verbose=0)
    pred4  = np.argmax(proba4, axis=1)

    print("label4 count:", len(idx4))
    print("label4 → predicted class distribution:", np.bincount(pred4, minlength=5))
else:
    print("⚠ label 4 샘플이 없습니다.")
