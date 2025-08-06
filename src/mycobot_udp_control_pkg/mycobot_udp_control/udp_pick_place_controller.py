import sys
import socket
import json
import time
import numpy as np
import threading
import argparse
from pymycobot.mycobot import MyCobot

# ───────────── 입력 인자 받기 ─────────────
parser = argparse.ArgumentParser()
parser.add_argument("target_id", type=int, help="목표 ArUco ID")
args = parser.parse_args()
TARGET_ID = args.target_id

# ───────────── 전역 설정 ─────────────
latest_pose = None
pose_lock = threading.Lock()

mc = MyCobot("/dev/ttyUSB0", 1000000)
print("로봇팔 연결됨")

initial_coords = mc.get_coords()
base_x, base_y, base_z = initial_coords[0], initial_coords[1], initial_coords[2]
print(f"기준 위치 → X:{base_x:.1f}, Y:{base_y:.1f}, Z:{base_z:.1f}")

# ────────── 파라미터 설정 ──────────
UDP_IP = "192.168.0.161"
UDP_PORT = 9999

P_GAIN = 900
STEP = 6.5
Z_STEP = 1.1
SPEED = 25
THRESHOLD = 0.005
Z_THRESHOLD = 0.007
MIN_MOVE = 0.2
DESIRED_DZ = 0.1
EMA_ALPHA = 0.2
Z_DELAY_SEC = 1.5
XY_STABLE_DURATION = 1.5

# Pick 관련 설정
GRIPPER_FORWARD_OFFSET = 42  # mm (4.2cm)
GRIPPER_DOWN_OFFSET = 55     # mm (5.5cm)

ema_dx, ema_dy, ema_dz = 0.0, 0.0, 0.0
ema_initialized = False
xy_aligned = False
xy_stable_start_time = None
z_started = False
pick_done = False

# ────────── Pick 동작 함수 ──────────
def perform_pick():
    print("Pick 동작 시작!")

    # 현재 위치 기준 좌표 저장
    base_coords = mc.get_coords()
    base_x = base_coords[0]
    base_y = base_coords[1]
    base_z = base_coords[2]

    pick_z = base_z - GRIPPER_DOWN_OFFSET  # mm → cm
    lift_z = pick_z + 50.0  # 집은 후 들어올릴 높이 (cm)

    # Pick 위치로 이동
    pick_coords = [base_x, base_y + GRIPPER_FORWARD_OFFSET, pick_z, 180, 0, 0]
    print(f"Pick 위치 이동: {pick_coords}")
    mc.send_coords(pick_coords, SPEED)
    time.sleep(2)

    # 그리퍼로 집기
    mc.set_gripper_value(50, 50)
    print("그리퍼 닫기")
    time.sleep(2)

    # 물체 들어올리기
    lift_coords = [base_x, base_y + GRIPPER_FORWARD_OFFSET, lift_z, 180, 0, 0]
    print(f"들어올림 위치 이동: {lift_coords}")
    mc.send_coords(lift_coords, SPEED)
    time.sleep(1)

    print("Pick 완료!")
    
    mc.send_angles([90, 0, -45, -45, 0, 0], 50)
    time.sleep(2.0)
    mc.send_angles([0, 0, 0, 0, 0, 0], 50)
    time.sleep(2.0)
    mc.send_angles([0, -60, -25, 50, 0, 0], 50)
    time.sleep(2.0)
    mc.set_gripper_value(100, 50)
    time.sleep(2.0)
    mc.send_angles([0, 0, 0, 0, 0, 0], 50)
    print("Pick & Place 완료")

# ────────── UDP 수신 스레드 ──────────
def udp_receiver():
    global latest_pose
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"UDP 수신 대기 중: {UDP_IP}:{UDP_PORT}")

    frame_count = 0
    start_time = time.time()

    while True:
        data, _ = sock.recvfrom(4096)
        try:
            pose_list = json.loads(data.decode())
            if pose_list and isinstance(pose_list, list):
                with pose_lock:
                    latest_pose = pose_list[0]

            frame_count += 1
            if time.time() - start_time >= 1.0:
                print(f"[RECEIVED FPS] {frame_count}")
                frame_count = 0
                start_time = time.time()
        except Exception as e:
            print(f"JSON 오류: {e}")

# ────────── 로봇 제어 스레드 ──────────
def robot_controller():
    global latest_pose, ema_dx, ema_dy, ema_dz
    global ema_initialized, xy_aligned, xy_stable_start_time, z_started, pick_done

    while True:
        with pose_lock:
            pose = latest_pose

        if not pose:
            time.sleep(0.01)
            continue

        marker_id = pose.get("id")
        tvec = pose.get("tvec")
        if marker_id is None or tvec is None:
            continue
        
        if marker_id != TARGET_ID:
            continue

        dx, dy, dz = tvec

        # EMA 필터 적용
        if not ema_initialized:
            ema_dx, ema_dy, ema_dz = dx, dy, dz
            ema_initialized = True
        else:
            ema_dx = EMA_ALPHA * dx + (1 - EMA_ALPHA) * ema_dx
            ema_dy = EMA_ALPHA * dy + (1 - EMA_ALPHA) * ema_dy
            ema_dz = EMA_ALPHA * dz + (1 - EMA_ALPHA) * ema_dz

        print(f"[EMA] dx={ema_dx:.4f}, dy={ema_dy:.4f}, dz={ema_dz:.4f}")

        # 중심 정렬 상태 판단
        x_done = abs(ema_dx) < THRESHOLD
        y_done = abs(ema_dy) < THRESHOLD
        z_error = ema_dz - DESIRED_DZ
        z_done = abs(z_error) < Z_THRESHOLD

        current_coords = mc.get_coords()
        current_x, current_y, current_z = current_coords[:3]

        # Step 1: XY 정렬
        if not xy_aligned:
            if x_done and y_done:
                if xy_stable_start_time is None:
                    xy_stable_start_time = time.time()
                    print("XY 정렬 안정화 타이머 시작")
                elif time.time() - xy_stable_start_time >= XY_STABLE_DURATION:
                    xy_aligned = True
                    print("XY 정렬 완료 → Z축 대기 시작")
                else:
                    print("XY 안정화 유지 중...")
                time.sleep(0.1)
                continue
            else:
                xy_stable_start_time = None

            offset_x = np.clip(ema_dx * P_GAIN, -STEP, STEP)
            offset_y = np.clip(-ema_dy * P_GAIN, -STEP, STEP)

            if abs(offset_x) < MIN_MOVE and abs(offset_y) < MIN_MOVE:
                print("XY 이동 생략 (미세)")
                time.sleep(0.05)
                continue

            target_x = current_x + offset_x
            target_y = current_y + offset_y
            target_z = base_z

            print(f"[Move XY] → X:{target_x:.1f}, Y:{target_y:.1f}, Z:{target_z:.1f}")
            mc.send_coords([target_x, target_y, target_z, 180, 0, 0], SPEED)

        # Step 2: Z축 보정 (XY 정렬 후 일정 시간 대기)
        elif not z_done:
            if not z_started:
                print("Z축 보정 대기 중...")
                time.sleep(Z_DELAY_SEC)
                z_started = True

            offset_z = -Z_STEP if z_error > 0 else Z_STEP
            if abs(offset_z) >= MIN_MOVE:
                target_z = current_z + offset_z
                print(f"[Move Z] → Z:{target_z:.1f}, z_error={z_error:.4f}")
                mc.send_coords([current_x, current_y, target_z, 180, 0, 0], SPEED)
            else:
                print("Z 이동 생략 (미세)")

        # Step 3: 중심 정렬 완료 후 Pick
        elif z_done and not pick_done:
            print("XYZ 정렬 완료 → Pick 시작")
            perform_pick()
            pick_done = True
            time.sleep(3)

        time.sleep(0.1)

# ────────── 메인 실행 ──────────
if __name__ == "__main__":
    threading.Thread(target=udp_receiver, daemon=True).start()
    threading.Thread(target=robot_controller, daemon=True).start()

    while True:
        time.sleep(1)