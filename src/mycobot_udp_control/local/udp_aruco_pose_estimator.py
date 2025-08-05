import socket
import cv2
import numpy as np
import json
import time

# ──────────────────────────────────────
# 설정
# ──────────────────────────────────────
# 1. UDP 수신 (영상 수신용)
UDP_IP = "192.168.0.153"
UDP_PORT = 5005

# 2. UDP 송신 (Pose 전송용)
PI_IP = "192.168.0.161"     # Raspberry Pi IP
PI_PORT = 9999

# 3. 카메라 보정 파일
camera_matrix = np.load("/home/addinedu/dev_ws/myCobot280/calibration_matrix.npy")
dist_coeffs = np.load("/home/addinedu/dev_ws/myCobot280/distortion_coefficients.npy")

print("Camera Matrix:\n", camera_matrix)
print("Distortion Coefficients:\n", dist_coeffs)

marker_length = 0.02  # [m]

# 4. ArUco 설정
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

# ──────────────────────────────────────
# 소켓 초기화
# ──────────────────────────────────────
recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv_sock.bind((UDP_IP, UDP_PORT))

send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print(f"Receiving video on port {UDP_PORT}...")
print(f"Sending pose to {PI_IP}:{PI_PORT}")

# FPS 측정용
frame_count = 0
start_time = time.time()

# FPS 제한용
target_fps = 30
frame_delay = 1.0 / target_fps
last_frame_time = time.time()

# ──────────────────────────────────────
# 메인 루프
# ──────────────────────────────────────
while True:
    data, addr = recv_sock.recvfrom(65536)  # 수신 버퍼 크기 설정
    np_data = np.frombuffer(data, dtype=np.uint8)
    frame = cv2.imdecode(np_data, cv2.IMREAD_COLOR)

    if frame is None:
        continue

    # ArUco 인식
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    pose_dict = {}
    if ids is not None:
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, marker_length, camera_matrix, dist_coeffs
        )

        for i, marker_id in enumerate(ids.flatten()):
            tvec = tvecs[i][0].tolist()
            pose_dict[int(marker_id)] = tvec

            # 시각화
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], marker_length * 0.5)

    # Pose UDP 전송 (LIST 형태로 변경)
    pose_list = [{"id": int(k), "tvec": v} for k, v in pose_dict.items()]
    try:
        json_data = json.dumps(pose_list)
        send_sock.sendto(json_data.encode(), (PI_IP, PI_PORT))
    except Exception as e:
        print(f"Send error: {e}")

    # 디버그 및 시각화
    for id, tvec in pose_dict.items():
        print(f"[ID {id}] x={tvec[0]:.4f}, y={tvec[1]:.4f}, z={tvec[2]:.4f}")
    
    cv2.imshow("ArUco Pose Estimator", frame)
    
    frame_count += 1
    elapsed = time.time() - start_time
    if elapsed >= 1.0:
        print(f"[FPS] {frame_count}")
        frame_count = 0
        start_time = time.time()
    
    now = time.time()
    delay = frame_delay - (now - last_frame_time)
    if delay > 0:
        time.sleep(delay)
    last_frame_time = time.time()
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 종료 처리
recv_sock.close()
send_sock.close()
cv2.destroyAllWindows()
