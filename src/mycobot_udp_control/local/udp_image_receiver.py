import socket
import cv2
import numpy as np
import time

UDP_IP = "192.168.0.153"  # 모든 인터페이스에서 수신
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening on {UDP_PORT} for incoming frames...")

frame_count = 0
start_time = time.time()

while True:
    data, addr = sock.recvfrom(65536)  # 64KB 이하 데이터 수신
    frame_count += 1
    
    np_data = np.frombuffer(data, dtype=np.uint8)
    frame = cv2.imdecode(np_data, cv2.IMREAD_COLOR)
    
    elapsed = time.time() - start_time
    if elapsed >= 1.0:
        print(f"[RECEIVED FPS] {frame_count}")
        frame_count = 0
        start_time = time.time()
    
    if frame is not None:
        cv2.imshow("UDP Camera Stream", frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

sock.close()
cv2.destroyAllWindows()
