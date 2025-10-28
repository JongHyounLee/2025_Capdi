# =========================
# 0) 기본 설정 & 라이브러리
# =========================
from google.colab import drive
drive.mount('/content/drive')

CSV_PATH  = "/content/drive/MyDrive/imu_data/IMU_Label_Plus.csv"  # 단일 CSV
CKPT_DIR  = "/content/drive/MyDrive/imu_runs/checkpoints"
LOG_DIR   = "/content/drive/MyDrive/imu_runs/logs"

import os, time
os.makedirs(CKPT_DIR, exist_ok=True)
os.makedirs(LOG_DIR,  exist_ok=True)

import numpy as np
import pandas as pd
import tensorflow as tf
from sklearn.model_selection import train_test_split
from sklearn.utils.class_weight import compute_class_weight

SEED = 42
np.random.seed(SEED)
tf.random.set_seed(SEED)

# =========================================
# 1) CSV → 윈도우 시퀀스 (move로 segment 구분)
# =========================================
SEQ_LEN   = 6
BATCH_SIZE= 64
EPOCHS    = 20
NUM_IMU   = 5
AXES      = ["ax","ay","az","gx","gy","gz"]   # 각 IMU에서 쓰는 6개 채널
NUM_FEATS = NUM_IMU * len(AXES)               # 5*6 = 30? → tick 제외. (아래에서 정확히 35가 되도록 체크)

def _to_int_series(series, prefix=None):
    """
    move/label 컬럼이 'move:3'/'label:2' 같이 문자열일 수도, 숫자일 수도 있으니
    둘 다 안전하게 int 배열로 변환.
    """
    if pd.api.types.is_numeric_dtype(series):
        return series.astype(int).to_numpy()
    s = series.astype(str)
    if prefix is not None:
        # 예: 'move:3' 형태에서도 작동
        return (s.str.extract(fr"{prefix}:(\d+)", expand=False).astype(int)).to_numpy()
    # 숫자만 남기기 시도
    return (s.str.extract(r"(\d+)", expand=False).astype(int)).to_numpy()

def load_single_csv_make_windows(csv_path, seq_len=6):
    # 헤더가 있는 CSV로 가정 (첫 줄이 컬럼명)
    df = pd.read_csv(csv_path, header=0, low_memory=False)

    # ---- feature 컬럼 선정 (tick 제외) ----
    # 원하는 순서: IMU1_ax..gz, IMU2_ax..gz, ... IMU5_ax..gz
    feature_cols = []
    for i in range(1, NUM_IMU+1):
        for ax in AXES:
            col = f"IMU{i}_{ax}"
            if col not in df.columns:
                raise ValueError(f"컬럼 {col} 이(가) CSV에 없음. 실제 컬럼들: {list(df.columns)}")
            feature_cols.append(col)

    # 보너스: tick 포함 여부 점검 (있어도 무시)
    # 예: 'IMU1_tick' ... 'IMU5_tick' 은 사용하지 않음
    feats = df[feature_cols].astype(np.float32).to_numpy()  # (T, num_features)

    # ---- move / label ----
    if "move" not in df.columns or "label" not in df.columns:
        raise ValueError("CSV에 'move', 'label' 컬럼이 필요합니다.")

    moves  = _to_int_series(df["move"],  prefix="move")
    labels = _to_int_series(df["label"], prefix="label")

    # ---- move가 변하는 지점에서 세그먼트 분리 ----
    boundaries = np.where(np.diff(moves) != 0)[0] + 1
    segments = []
    start = 0
    for b in boundaries:
        segments.append((start, b))
        start = b
    segments.append((start, len(df)))

    # ---- 슬라이딩 윈도우 만들기 ----
    X_list, Y_list = [], []
    for (s, e) in segments:
        seg_len = e - s
        if seg_len < seq_len:
            continue
        for t in range(s + seq_len - 1, e):
            window = feats[t - seq_len + 1 : t + 1]   # (seq_len, num_features)
            X_list.append(window)
            Y_list.append(labels[t])

    if len(X_list) == 0:
        raise RuntimeError(
            f"윈도우 생성 실패: seq_len={seq_len}이 너무 길거나, 유효 segment가 없음."
        )

    X_np = np.stack(X_list).astype(np.float32)         # (N, seq_len, num_features)
    y_np = np.array(Y_list, dtype=np.int64)            # (N,)
    return X_np, y_np, feature_cols

X_all, y_all, feature_cols = load_single_csv_make_windows(CSV_PATH, seq_len=SEQ_LEN)

print("X_all:", X_all.shape)      # (N, SEQ_LEN, num_features)
print("y_all:", y_all.shape)      # (N,)
print("unique labels:", np.unique(y_all))
print("num_features:", X_all.shape[2])
assert set(np.unique(y_all)).issubset({0,1,2,3,4}), "라벨이 0~4 범위를 벗어납니다."

# ======================================
# 2) train/val split + tf.data 파이프라인
# ======================================
num_classes = 5
X_train, X_val, y_train, y_val = train_test_split(
    X_all, y_all, test_size=0.2, shuffle=True, stratify=y_all, random_state=SEED
)

train_ds = tf.data.Dataset.from_tensor_slices((X_train, y_train)) \
    .shuffle(len(X_train), seed=SEED) \
    .batch(BATCH_SIZE) \
    .prefetch(tf.data.AUTOTUNE)

val_ds = tf.data.Dataset.from_tensor_slices((X_val, y_val)) \
    .batch(BATCH_SIZE) \
    .prefetch(tf.data.AUTOTUNE)

# (선택) 클래스 불균형 보정 가중치
classes = np.unique(y_all)
class_weights = compute_class_weight(class_weight="balanced", classes=classes, y=y_all)
class_weight_dict = {int(c): float(w) for c, w in zip(classes, class_weights)}
print("class_weight:", class_weight_dict)

# ===========================
# 3) Keras TCN 블록 & 모델
# ===========================
from tensorflow.keras import layers, models

class TemporalBlock(layers.Layer):
    def __init__(self, out_channels, kernel_size, dilation_rate, dropout):
        super().__init__()
        self.out_channels = out_channels
        self.kernel_size = kernel_size
        self.dilation_rate = dilation_rate
        self.dropout_rate = dropout

        self.conv1 = layers.Conv1D(
            filters=out_channels,
            kernel_size=kernel_size,
            dilation_rate=dilation_rate,
            padding='causal'
        )
        self.relu1 = layers.ReLU()
        self.drop1 = layers.Dropout(dropout)

        self.conv2 = layers.Conv1D(
            filters=out_channels,
            kernel_size=kernel_size,
            dilation_rate=dilation_rate,
            padding='causal'
        )
        self.relu2 = layers.ReLU()
        self.drop2 = layers.Dropout(dropout)

        self.downsample = None  # build()에서 채널 수 맞추기

    def build(self, input_shape):
        in_channels = input_shape[-1]
        if in_channels != self.out_channels:
            self.downsample = layers.Conv1D(filters=self.out_channels, kernel_size=1, padding='same')

    def call(self, x, training=False):
        out = self.conv1(x)
        out = self.relu1(out)
        out = self.drop1(out, training=training)

        out = self.conv2(out)
        out = self.relu2(out)
        out = self.drop2(out, training=training)

        res = x if self.downsample is None else self.downsample(x)
        return tf.nn.relu(out + res)

def build_tcn_classifier(
    seq_len,
    num_features,
    num_classes,
    tcn_channels=(64, 128, 256),
    kernel_size=3,
    dropout=0.2,
    use_global_avg_pool=False
):
    inp = layers.Input(shape=(seq_len, num_features))  # (B, T, C)

    x = inp
    for i, ch in enumerate(tcn_channels):
        dilation = 2 ** i
        x = TemporalBlock(ch, kernel_size, dilation, dropout)(x)

    if use_global_avg_pool:
        x = layers.GlobalAveragePooling1D()(x)          # (B, C_last)
    else:
        x = layers.Lambda(lambda t: t[:, -1, :])(x)     # 마지막 타임스텝만

    logits = layers.Dense(num_classes)(x)
    out = layers.Activation('softmax')(logits)
    model = models.Model(inputs=inp, outputs=out)
    return model

seq_len      = X_all.shape[1]
num_features = X_all.shape[2]

model = build_tcn_classifier(
    seq_len=seq_len,
    num_features=num_features,
    num_classes=num_classes,
    tcn_channels=(64, 128, 256),
    kernel_size=3,
    dropout=0.2,
    use_global_avg_pool=False
)
model.compile(
    optimizer=tf.keras.optimizers.Adam(1e-3),
    loss="sparse_categorical_crossentropy",
    metrics=["accuracy"]
)
model.summary()

# ====================
# 4) 학습 & 저장 설정
# ====================
stamp = time.strftime("%Y%m%d_%H%M%S")

ckpt_path = os.path.join(CKPT_DIR, f"best_{stamp}.keras")  # Keras v3 포맷
callbacks = [
    tf.keras.callbacks.ModelCheckpoint(
        ckpt_path, monitor="val_accuracy", mode="max",
        save_best_only=True, save_weights_only=False, verbose=1
    ),
    tf.keras.callbacks.EarlyStopping(
        monitor="val_accuracy", mode="max",
        patience=8, restore_best_weights=True, verbose=1
    ),
    tf.keras.callbacks.TensorBoard(
        log_dir=os.path.join(LOG_DIR, f"tcn_{stamp}"))
]

history = model.fit(
    train_ds,
    validation_data=val_ds,
    epochs=EPOCHS,
    callbacks=callbacks,
    class_weight=class_weight_dict  # 불균형 없으면 제거해도 됨
)

# ==========
# 5) 평가/추론
# ==========
val_loss, val_acc = model.evaluate(val_ds, verbose=0)
print(f"[VAL] loss={val_loss:.4f}, acc={val_acc:.4f}")

probs = model.predict(X_all[0:1])
pred  = probs.argmax(axis=1)[0]
print("GT:", int(y_all[0]), "Pred:", int(pred), "probs:", probs[0])
print("Best model saved to:", ckpt_path)
