import serial
import threading
import tkinter as tk
from tkinter import messagebox
import os
import csv
from datetime import datetime
import re

# ====== 설정 ======
PORT = "COM3"          # 시리얼 포트
BAUD = 921600          # 보드레이트
SAVE_DIR = r"C:\Users\hjl99\Desktop\과제\캡디"

os.makedirs(SAVE_DIR, exist_ok=True)
SAVE_PATH = os.path.join(SAVE_DIR, f"IMU_LOG_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")

# ====== 고정 칼럼 정의 ======
COLS = ["Timestep"]
for k in range(1, 6):
    COLS += [f"IMU{k}_tick", f"IMU{k}_ax", f"IMU{k}_ay", f"IMU{k}_az",
             f"IMU{k}_gx", f"IMU{k}_gy", f"IMU{k}_gz"]
COLS += ["move", "label"]

# ====== 전역 ======
stop_flag = False
fh = None
writer = None
line_count = 0
lock = threading.Lock()  # 파일 쓰기 보호

# ====== 한 줄 파서 (튼튼하게) ======
num = r'(-?\d+)'  # 음수 포함 정수
def parse_line(line: str):
    """
    UART 한 줄 → dict (COLS 순서 채움, 누락은 '')
    """
    # 앞뒤 공백/탭/쉼표 정리
    s = line.strip().rstrip(",").replace("\t", " ")
    if not s:
        return None

    row = {c: "" for c in COLS}  # 기본은 빈칸

    # 1) Timestep
    mT = re.search(r'\bT\s*:\s*(\d+)', s)
    if mT:
        row["Timestep"] = mT.group(1)

    # 2) 각 IMU 블록 (정확히 7개 숫자)
    for k in range(1, 6):
        # IMU{n}, tick, ax, ay, az, gx, gy, gz
        # 콤마 사이 공백 허용
        pat = rf'IMU{k}\s*,\s*{num}\s*,\s*{num}\s*,\s*{num}\s*,\s*{num}\s*,\s*{num}\s*,\s*{num}\s*,\s*{num}'
        m = re.search(pat, s)
        if m:
            vals = m.groups()  # 7개
            keys = [f"IMU{k}_tick", f"IMU{k}_ax", f"IMU{k}_ay", f"IMU{k}_az",
                    f"IMU{k}_gx", f"IMU{k}_gy", f"IMU{k}_gz"]
            for key, val in zip(keys, vals):
                row[key] = val
        else:
            # 못 찾았으면 경고만 남기고 계속 (라인은 저장)
            print(f"[WARN] IMU{k} block missing/invalid → {line}")

    # 3) move / label (어떤 공백/쉼표 조합이든 허용)
    mm = re.search(r'\bmove\s*:\s*(-?\d+)', s)
    ml = re.search(r'\blabel\s*:\s*(-?\d+)', s)
    if mm:
        row["move"] = mm.group(1)
    else:
        print(f"[WARN] move missing → {line}")

    if ml:
        row["label"] = ml.group(1)
    else:
        print(f"[WARN] label missing → {line}")

    # 최소한 Timestep 이라도 있으면 저장
    if row["Timestep"] == "":
        print(f"[SKIP] invalid line (no Timestep) → {line}")
        return None

    return row

# ====== 시리얼 수신 스레드 ======
def read_serial():
    global stop_flag, fh, writer, line_count
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        print(f"[INFO] Logging started ({PORT} @ {BAUD}) → {SAVE_PATH}")

        # 파일 열고 헤더 작성 (BOM 포함 UTF-8, 엑셀 호환)
        with lock:
            fh = open(SAVE_PATH, "w", newline="", encoding="utf-8-sig")
            writer = csv.DictWriter(fh, fieldnames=COLS)
            writer.writeheader()
            fh.flush()

        while not stop_flag:
            if ser.in_waiting:
                # \n까지 읽되, CRLF/공백은 strip
                raw = ser.readline().decode(errors="ignore").strip()
                if not raw:
                    continue

                print(raw)  # 콘솔 출력

                row = parse_line(raw)
                if row:
                    with lock:
                        writer.writerow(row)
                        line_count += 1
                        # 너무 자주 flush하면 성능 나빠짐 → 주기적으로만
                        if line_count % 50 == 0:
                            fh.flush()

        ser.close()
        print("[INFO] Serial port closed.")

    except serial.SerialException as e:
        messagebox.showerror("Serial Error", str(e))
    except Exception as e:
        messagebox.showerror("Error", str(e))
    finally:
        with lock:
            if fh:
                fh.flush()
                fh.close()
                fh = None
                writer = None
        print(f"[INFO] Saved {line_count} lines → {SAVE_PATH}")

# ====== 종료 버튼 ======
def stop_logging():
    global stop_flag
    stop_flag = True
    root.destroy()

# ====== GUI ======
root = tk.Tk()
root.title("IMU Logger")
root.geometry("300x100")
tk.Button(root, text="종료", font=("Arial", 14), width=10, command=stop_logging).pack(pady=25)

# ====== 스레드 시작 & GUI 루프 ======
threading.Thread(target=read_serial, daemon=True).start()
root.mainloop()
