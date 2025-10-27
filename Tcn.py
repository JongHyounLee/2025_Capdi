from google.colab import drive
drive.mount('/content/drive')

# 단일 CSV 전체 경로로 바꿔줘
CSV_PATH  = "/content/drive/MyDrive/imu_data/IMU_Label_Plus.csv"

CKPT_DIR  = "/content/drive/MyDrive/imu_runs/checkpoints"
LOG_DIR   = "/content/drive/MyDrive/imu_runs/logs"

import os
os.makedirs(CKPT_DIR, exist_ok=True)
os.makedirs(LOG_DIR,  exist_ok=True)

import numpy as np
import pandas as pd
import torch

def load_single_csv_make_windows(csv_path, seq_len=6):
    df = pd.read_csv(csv_path, header=None)

    feature_cols = (
        list(range(2, 9)) +
        list(range(10, 17)) +
        list(range(18, 25)) +
        list(range(26, 33)) +
        list(range(34, 41))
    )
    feats = df[feature_cols].to_numpy(dtype=np.float32)  # (T, num_features)

    moves = (
        df[41]
        .astype(str)
        .str.extract(r"move:(\d+)", expand=False)
        .astype(int)
        .to_numpy()
    )
    labels = (
        df[42]
        .astype(str)
        .str.extract(r"label:(\d+)", expand=False)
        .astype(int)
        .to_numpy()
    )

    boundaries = np.where(np.diff(moves) != 0)[0] + 1
    segments = []
    start = 0
    for b in boundaries:
        segments.append((start, b))   # [start, b-1] 은 같은 move
        start = b
    segments.append((start, len(df))) # 마지막 구간 추가

    X_list = []
    Y_list = []
    for (s, e) in segments:
        seg_len = e - s
        if seg_len < seq_len:
            continue  # 이 구간은 너무 짧음
        for t in range(s + seq_len - 1, e):
            window = feats[t - seq_len + 1 : t + 1]  # (seq_len, num_features)
            X_list.append(window)
            Y_list.append(labels[t])

    if len(X_list) == 0:
        raise RuntimeError(
            f"윈도우를 못 만들었어. seq_len={seq_len}이(가) 너무 김 "
            f"또는 CSV 안에 유효 segment가 없음."
        )

    X_np = np.stack(X_list)                # (num_windows, seq_len, num_features)
    y_np = np.array(Y_list, dtype=np.int64)

    return X_np, y_np

# 여기 ↓↓↓가 핵심 (DATA_DIR 아님, CSV_PATH 넣기)
SEQ_LEN = 6
X_all, y_all = load_single_csv_make_windows(CSV_PATH, seq_len=SEQ_LEN)

print("X_all shape:", X_all.shape)
print("y_all shape:", y_all.shape)
print("unique labels:", np.unique(y_all))


from torch.utils.data import Dataset, DataLoader, Subset
import torch
import numpy as np

class IMUDataset(Dataset):
    def __init__(self, X, y):
        # X: (N, seq_len, num_features)  e.g. (N, 6, 35)
        # y: (N,)
        if isinstance(X, torch.Tensor):
            self.X = X.float()
        else:
            self.X = torch.tensor(X, dtype=torch.float32)

        if isinstance(y, torch.Tensor):
            self.y = y.long()
        else:
            self.y = torch.tensor(y, dtype=torch.long)

    def __len__(self):
        return self.X.shape[0]

    def __getitem__(self, idx):
        # 현재 x: (seq_len, num_features)
        x = self.X[idx]            # (T, C)
        x = x.transpose(0, 1)      # -> (C, T) = (num_features, seq_len)
        y = self.y[idx]            # scalar label
        return x, y
dataset = IMUDataset(X_all, y_all)

N = len(dataset)
indices = torch.randperm(N)
train_size = int(N * 0.8)
train_idx = indices[:train_size]
val_idx   = indices[train_size:]

train_ds = Subset(dataset, train_idx)
val_ds   = Subset(dataset, val_idx)

train_loader = DataLoader(train_ds, batch_size=64, shuffle=True, num_workers=2, pin_memory=True)
val_loader   = DataLoader(val_ds, batch_size=64, shuffle=False, num_workers=2, pin_memory=True)
import torch.nn as nn

class Chomp1d(nn.Module):
    def __init__(self, chomp_size: int):
        super().__init__()
        self.chomp_size = chomp_size

    def forward(self, x):
        # chomp_size만큼 오른쪽 잘라 causality 유지
        return x[:, :, :-self.chomp_size].contiguous() if self.chomp_size > 0 else x


class TemporalBlock(nn.Module):
    def __init__(self, in_channels, out_channels, kernel_size, dilation, dropout, padding_mode="zeros"):
        super().__init__()
        padding = (kernel_size - 1) * dilation

        self.conv1 = nn.Conv1d(
            in_channels,
            out_channels,
            kernel_size,
            padding=padding,
            dilation=dilation,
            padding_mode=padding_mode,
        )
        self.chomp1 = Chomp1d(padding)
        self.relu1 = nn.ReLU()
        self.dropout1 = nn.Dropout(dropout)

        self.conv2 = nn.Conv1d(
            out_channels,
            out_channels,
            kernel_size,
            padding=padding,
            dilation=dilation,
            padding_mode=padding_mode,
        )
        self.chomp2 = Chomp1d(padding)
        self.relu2 = nn.ReLU()
        self.dropout2 = nn.Dropout(dropout)

        self.downsample = nn.Conv1d(in_channels, out_channels, kernel_size=1) \
                          if in_channels != out_channels else None
        self.final_relu = nn.ReLU()

    def forward(self, x):
        out = self.conv1(x)
        out = self.chomp1(out)
        out = self.relu1(out)
        out = self.dropout1(out)

        out = self.conv2(out)
        out = self.chomp2(out)
        out = self.relu2(out)
        out = self.dropout2(out)

        res = x if self.downsample is None else self.downsample(x)
        return self.final_relu(out + res)


class TemporalConvNet(nn.Module):
    def __init__(self, in_channels, num_channels_list, kernel_size=3, dropout=0.2):
        super().__init__()
        layers = []
        for i, out_ch in enumerate(num_channels_list):
            dilation = 2 ** i
            in_ch = in_channels if i == 0 else num_channels_list[i - 1]
            layers.append(
                TemporalBlock(
                    in_channels=in_ch,
                    out_channels=out_ch,
                    kernel_size=kernel_size,
                    dilation=dilation,
                    dropout=dropout,
                )
            )
        self.network = nn.Sequential(*layers)

    def forward(self, x):
        # x: (B, C_in, T)
        return self.network(x)  # (B, C_out_last, T)


class IMU_TCN_Classifier(nn.Module):
    def __init__(
        self,
        num_features,    # e.g. 35
        num_classes,     # e.g. 4
        tcn_channels=(64, 128, 256),
        kernel_size=3,
        dropout=0.2,
        use_global_avg_pool=False,
    ):
        super().__init__()
        self.tcn = TemporalConvNet(
            in_channels=num_features,
            num_channels_list=tcn_channels,
            kernel_size=kernel_size,
            dropout=dropout,
        )
        self.use_global_avg_pool = use_global_avg_pool

        last_channels = tcn_channels[-1]
        self.classifier = nn.Linear(last_channels, num_classes)

    def forward(self, x):
        # x: (B, num_features, seq_len)
        feats = self.tcn(x)  # (B, C_last, T)

        if self.use_global_avg_pool:
            feats_pool = feats.mean(dim=-1)         # (B, C_last)
        else:
            feats_pool = feats[:, :, -1]            # (B, C_last)

        logits = self.classifier(feats_pool)        # (B, num_classes)
        return logits
import torch
import torch.optim as optim
from torch.cuda.amp import GradScaler, autocast

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
print("device:", device)

num_features = dataset[0][0].shape[0]  # (=35)
num_classes  = len(torch.unique(dataset.y))  # 예: 4개의 동작 라벨

model = IMU_TCN_Classifier(
    num_features=num_features,
    num_classes=num_classes,
    tcn_channels=(64, 128, 256),
    kernel_size=3,
    dropout=0.2,
    use_global_avg_pool=False,
).to(device)

criterion = nn.CrossEntropyLoss()
optimizer = optim.Adam(model.parameters(), lr=1e-3)
scaler = GradScaler(enabled=(device.type == 'cuda'))

EPOCHS = 20

for epoch in range(EPOCHS):
    ########################
    # Train
    ########################
    model.train()
    running_loss = 0.0
    running_correct = 0
    running_total = 0

    for batch_x, batch_y in train_loader:
        batch_x = batch_x.to(device, non_blocking=True)  # (B, C, T)
        batch_y = batch_y.to(device, non_blocking=True)  # (B,)

        optimizer.zero_grad(set_to_none=True)

        with autocast(enabled=(device.type == 'cuda')):
            logits = model(batch_x)          # (B, num_classes)
            loss = criterion(logits, batch_y)

        scaler.scale(loss).backward()
        scaler.step(optimizer)
        scaler.update()

        running_loss += loss.item() * batch_x.size(0)
        running_correct += (logits.argmax(dim=1) == batch_y).sum().item()
        running_total += batch_y.size(0)

    train_loss = running_loss / running_total
    train_acc  = running_correct / running_total

    ########################
    # Validation
    ########################
    model.eval()
    val_loss_sum = 0.0
    val_correct  = 0
    val_total    = 0

    with torch.no_grad():
        for batch_x, batch_y in val_loader:
            batch_x = batch_x.to(device, non_blocking=True)
            batch_y = batch_y.to(device, non_blocking=True)

            logits = model(batch_x)
            loss = criterion(logits, batch_y)

            val_loss_sum += loss.item() * batch_x.size(0)
            val_correct  += (logits.argmax(dim=1) == batch_y).sum().item()
            val_total    += batch_y.size(0)

    val_loss = val_loss_sum / val_total
    val_acc  = val_correct / val_total

    print(f"[Epoch {epoch+1}] train_loss={train_loss:.4f} "
          f"train_acc={train_acc:.4f} val_loss={val_loss:.4f} val_acc={val_acc:.4f}")

    # 체크포인트(모델 가중치) 드라이브에 저장
    ckpt_path = f"{CKPT_DIR}/epoch_{epoch+1:03d}.pt"
    torch.save({
        "epoch": epoch + 1,
        "model_state_dict": model.state_dict(),
        "optimizer_state_dict": optimizer.state_dict(),
    }, ckpt_path)
model.eval()
with torch.no_grad():
    x_sample, y_sample = dataset[0]       # x_sample: (C, T)
    x_sample = x_sample.unsqueeze(0).to(device)  # (1, C, T)
    logits = model(x_sample)              # (1, num_classes)
    pred_class = torch.argmax(logits, dim=1).item()

print("GT label:", int(y_sample), "Pred:", pred_class)
