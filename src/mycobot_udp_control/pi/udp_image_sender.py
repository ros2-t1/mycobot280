import cv2
import socket
import numpy as np
import time
import subprocess

# UDP 서버 설정
UDP_IP = "192.168.0.153"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ────────────────────────────────────────────────
def kill_conflicting_camera_processes():
    try:
        output = subprocess.check_output("fuser /dev/video0", shell=True)
        pids = output.decode().strip().split()
        for pid in pids:
            print(f"카메라 점유 프로세스 종료: PID={pid}")
            subprocess.call(["kill", "-9", pid])
    except subprocess.CalledProcessError:
        print("충돌 중인 프로세스 없음")

def find_available_camera(max_index=5):
    for i in range(max_index):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            print(f"카메라 장치 발견: /dev/video{i}")
            return cap
        cap.release()
    return None

def initialize_camera(retry=5, delay=1):
    for i in range(retry):
        cap = find_available_camera()
        if cap and cap.isOpened():
            print("카메라 연결 성공")
            return cap
        print(f"카메라 연결 실패 ({i+1}/{retry}) → {delay}초 후 재시도")
        time.sleep(delay)
    print("최종적으로 카메라 연결 실패. 프로그램 종료.")
    exit()

# ────────────────────────────────────────────────
kill_conflicting_camera_processes()
cap = initialize_camera()

print(f"Streaming video to {UDP_IP}:{UDP_PORT}")

frame_count = 0
start_time = time.time()

# ────────────────────────────────────────────────
while True:
    if not cap.isOpened():
        print("스트림 종료 감지 → 카메라 재연결 시도")
        cap.release()
        cap = initialize_camera()

    ret, frame = cap.read()
    if not ret:
        print("프레임 수신 실패 → 카메라 재연결 시도")
        cap.release()
        cap = initialize_camera()
        continue

    frame = cv2.resize(frame, (640, 480))
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
    result, img_encoded = cv2.imencode('.jpg', frame, encode_param)

    if not result:
        print("이미지 인코딩 실패")
        continue

    data = np.array(img_encoded).tobytes()

    try:
        sock.sendto(data, (UDP_IP, UDP_PORT))
        frame_count += 1
    except socket.error as e:
        print(f"Socket Error: {e}")
        break

    elapsed = time.time() - start_time
    if elapsed >= 1.0:
        print(f"[FPS] {frame_count}")
        frame_count = 0
        start_time = time.time()

    time.sleep(1 / 30)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("사용자 요청 종료")
        break

cap.release()
sock.close()
cv2.destroyAllWindows()
