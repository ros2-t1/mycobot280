import socket
import json
import time

# ─────────────────────────────────────────────
# UDP 수신 설정
# ─────────────────────────────────────────────
UDP_IP = "192.168.0.161"
UDP_PORT = 9999

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening on UDP {UDP_PORT} for ArUco pose data...")

# FPS 측정 변수
frame_count = 0
start_time = time.time()

# ─────────────────────────────────────────────
# 메인 루프
# ─────────────────────────────────────────────
while True:
    data, addr = sock.recvfrom(4096)
    try:
        pose_list = json.loads(data.decode())

        if not pose_list or not isinstance(pose_list, list):
            print("No marker detected.")
            continue

        for pose in pose_list:
            marker_id = pose.get("id")
            tvec = pose.get("tvec")

            if marker_id is None or tvec is None:
                continue

            x, y, z = tvec
            print(f"[ID {marker_id}] x={x:.4f}, y={y:.4f}, z={z:.4f}")

        # FPS 계산
        frame_count += 1
        elapsed = time.time() - start_time
        if elapsed >= 1.0:
            print(f"[RECEIVED FPS] {frame_count}")
            frame_count = 0
            start_time = time.time()

    except json.JSONDecodeError as e:
        print(f"JSON decode error: {e}")
