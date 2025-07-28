import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import subprocess
import os
import signal
import time

class ArucoLauncher(Node):
    def __init__(self):
        super().__init__('aruco_launcher')
        self.target_id = None
        self.sub_id = self.create_subscription(Int32, '/target_aruco_id', self.id_callback, 10)
        self.sub_cmd = self.create_subscription(String, '/aruco_start', self.start_callback, 10)
        self.get_logger().info("ArUco 런처 노드 활성화됨")

    def id_callback(self, msg):
        self.target_id = msg.data
        self.get_logger().info(f"타겟 ArUco ID 수신: {self.target_id}")

    def start_callback(self, msg):
        if msg.data != "start":
            self.get_logger().warn("알 수 없는 명령어")
            return

        if self.target_id is None:
            self.get_logger().warn("아직 /target_aruco_id 토픽이 수신되지 않았습니다.")
            return

        self.get_logger().info("ArUco 처리 시작")

        # Flask 서버 실행 여부 확인
        if not self.is_port_in_use(5000):
            subprocess.Popen(["python3", "/home/jetcobot/dev_ws/src/video_stream_aruco_pose.py"])
            self.get_logger().info("Flask 서버 실행")
            time.sleep(2)
        else:
            self.get_logger().info("Flask 서버 이미 실행 중 → 생략")

        # Pick & Place 수행
        self.get_logger().info(f"ArUco 중심 정렬 및 Pick 시작 (ID {self.target_id})")
        subprocess.Popen(["python3", "/home/jetcobot/dev_ws/src/aruco_robot_centering.py", str(self.target_id)])

    def is_port_in_use(self, port):
        import socket
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            return s.connect_ex(("localhost", port)) == 0

def main(args=None):
    rclpy.init(args=args)
    node = ArucoLauncher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
