import sys
import requests
import time
import numpy as np
from pymycobot.mycobot import MyCobot

# ID 인자 확인
if len(sys.argv) != 2:
    print("사용법: python3 aruco_robot_centering.py <target_id>")
    sys.exit(1)

target_id = int(sys.argv[1])
print(f"감지 대상 마커 ID: {target_id}")

# 로봇팔 초기화
mc = MyCobot('/dev/ttyUSB0', 1000000)
print("로봇팔 연결됨")

# 파라미터
FLASK_URL = "http://192.168.0.161:5000/pose"
SCALE = 1000
STEP, Z_STEP = 10, 10
THRESHOLD, Z_THRESHOLD = 0.003, 0.01
SAFE_DIST = 0.1
SPEED = 50
base_z = 256.6
GRIPPER_FORWARD_OFFSET = 42
GRIPPER_DOWN_OFFSET = 55

while True:
    try:
        current_coords = mc.get_coords()
        base_x, base_y, base_z_now = current_coords[0], current_coords[1], current_coords[2]
        res = requests.get(FLASK_URL).json()
        tvec_dict = res

        if str(target_id) not in tvec_dict:
            print(f"ID {target_id} 감지 안됨 → 대기 중")
            time.sleep(0.5)
            continue

        for id_str, tvec in tvec_dict.items():
            if str(id_str) != str(target_id):
                print(f"ID {id_str} 감지되었지만 대상이 아님 → 무시")
                continue

            offset_x, offset_y, offset_z = tvec
            print(f"[Tvec] x: {offset_x:.4f}, y: {offset_y:.4f}, z: {offset_z:.4f}")

            move_required = False
            target_x = base_x + np.clip(offset_x * SCALE, -STEP, STEP) if abs(offset_x) > THRESHOLD else base_x
            target_y = base_y + np.clip(-offset_y * SCALE, -STEP, STEP) if abs(offset_y) > THRESHOLD else base_y
            target_z = base_z_now - np.clip((offset_z - SAFE_DIST) * SCALE, -Z_STEP, Z_STEP) if abs(offset_z - SAFE_DIST) > Z_THRESHOLD else base_z_now

            if (target_x != base_x) or (target_y != base_y) or (target_z != base_z_now):
                print(f"[Move] → X: {target_x:.1f}, Y: {target_y:.1f}, Z: {target_z:.1f}")
                mc.send_coords([target_x, target_y, target_z, 180, 0, 0], SPEED)
                time.sleep(1.0)
            else:
                print("정렬 상태 확인 → aligned")
                time.sleep(1.0)

                # Pick을 위한 현재 위치 저장 및 계산
                base_coords = mc.get_coords()
                base_x = base_coords[0]
                base_y = base_coords[1]
                pick_z = base_coords[2] - GRIPPER_DOWN_OFFSET
                lift_z = pick_z + 150  # 필요 시 100~150 사이로 조절

                # Pick 위치로 이동
                pick_coords = [base_x, base_y + GRIPPER_FORWARD_OFFSET, pick_z, 180, 0, 0]
                mc.send_coords(pick_coords, SPEED)
                time.sleep(2)

                # 그리퍼로 집기
                mc.set_gripper_value(50, 50)
                time.sleep(1)

                # # 들어올리기
                # lift_coords = pick_coords.copy()
                # lift_coords[2] = lift_z
                # mc.send_coords(lift_coords, SPEED)
                # time.sleep(2)
                
                mc.send_angles([90, 0, -45, -45, 0, 0], 50)
                time.sleep(2)
                
                mc.send_angles([0, 0, 0, 0, 0, 0], 50)
                time.sleep(1.5)
                mc.send_angles([0, -60, -25, 0, 0, 0], 50)
                time.sleep(1.5)
                mc.set_gripper_value(100, 50)
                time.sleep(1)

                mc.send_angles([0, 0, 0, 0, 0, 0], 50)
                print("Pick & Place 완료")
                sys.exit(0)

    except KeyboardInterrupt:
        print("사용자 종료")
        break
