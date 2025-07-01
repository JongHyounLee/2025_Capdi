import cv2
import mediapipe as mp
import pandas as pd
from scipy.signal import savgol_filter

print("🔍 스켈레톤 추적 시작 준비...")

# Pose 모델 초기화
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()
cap = cv2.VideoCapture(0)

# 웹캠 확인
if not cap.isOpened():
    print("❌ 웹캠을 열 수 없습니다.")
    exit()

print("✅ 웹캠 열기 성공!")

# 저장 변수 초기화
output_data = []
frame_index = 0           # 전체 프레임 수
save_interval = 5        # 저장 간격: 30프레임에 1회 저장 (1초마다)

while True:
    ret, frame = cap.read()
    if not ret:
        print("⚠️ 프레임을 읽을 수 없습니다.")
        break

    frame_index += 1

    img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    result = pose.process(img_rgb)

    if result.pose_landmarks:
        # 저장 조건 (간격마다만 저장)
        if frame_index % save_interval == 0:
            print(f"💾 저장 중: 프레임 {frame_index}")
            joints = []
            for lm in result.pose_landmarks.landmark:
                joints.extend([lm.x, lm.y, lm.z, lm.visibility])
            output_data.append(joints)

        # 관절 랜드마크 시각화
        mp.solutions.drawing_utils.draw_landmarks(
            frame, result.pose_landmarks, mp_pose.POSE_CONNECTIONS
        )

    # 실시간 화면 표시
    cv2.imshow("🦴 실시간 스켈레톤 추적 ('q' 키로 종료)", frame)

    # 종료 조건
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("🛑 'q' 키 입력으로 종료합니다.")
        break

# 필터 파라미터 설정
window_length = 9   # 반드시 홀수, 5~15 권장
polyorder = 3       # 보통 2~4 사용
# 저장
columns = []
for i in range(33):
    columns.extend([f'{i}_x', f'{i}_y', f'{i}_z', f'{i}_vis'])

df = pd.DataFrame(output_data, columns=columns)

# 📦 Savitzky-Golay 필터 후처리
from scipy.signal import savgol_filter
window_length = 9
polyorder = 3
for col in df.columns:
    try:
        df[col] = savgol_filter(df[col], window_length=window_length, polyorder=polyorder)
    except ValueError:
        print(f"⚠️ 필터 생략: '{col}' (프레임 수 부족)")

df.to_csv("skeleton_data.csv", index=False)
print("✅ skeleton_data.csv 저장 완료 (필터 적용됨).")
