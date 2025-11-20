import pandas as pd
import glob
import os

# ====== 경로 설정 ======
DATA_DIR = r"C:\Users\hjl99\Desktop\IMU_MODEL_DATA\labe"
SAVE_PATH = os.path.join(DATA_DIR, "imu_label_4_ALL.csv")

# ====== 병합할 파일 불러오기 ======
csv_files = sorted(glob.glob(os.path.join(DATA_DIR, "IMU_label_*.csv")))
print(f"총 {len(csv_files)}개 파일 발견:")
for f in csv_files:
    print(" -", os.path.basename(f))

# ====== 병합 과정 ======
all_dfs = []
move_offset = 0

for path in csv_files:
    df = pd.read_csv(path, header=0)

    # move / label 숫자 추출 ('move:3' 같은 문자열 방어)
    df["move"] = df["move"].astype(str).str.extract(r"(\d+)", expand=False).astype(int)
    df["label"] = df["label"].astype(str).str.extract(r"(\d+)", expand=False).astype(int)

    # move 번호를 전역적으로 이어지게 누적 offset 적용
    df["move"] = df["move"] + move_offset
    move_offset = df["move"].max()

    # 사람 구분용 이름 (파일명에서 추출)
    person_name = os.path.splitext(os.path.basename(path))[0]
    df["person_id"] = person_name

    all_dfs.append(df)

# ====== 모든 파일 합치기 ======
merged_df = pd.concat(all_dfs, ignore_index=True)

# ====== CSV 저장 ======
merged_df.to_csv(SAVE_PATH, index=False, encoding="utf-8-sig")
print("\n✅ 병합 완료:", SAVE_PATH)
print("총 행 수:", len(merged_df))
print("최대 move 번호:", merged_df['move'].max())
print("사람 수:", merged_df['person_id'].nunique())
print("person_id 목록:", merged_df['person_id'].unique())
