import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import subprocess
import os
import time

class ArucoUDPLauncher(Node):
    def __init__(self):
        super().__init__('aruco_udp_launcher')
        self.target_id = None
        self.sub_id = self.create_subscription(Int32, '/aruco_target_id', self.id_callback, 10)
        self.sub_cmd = self.create_subscription(String, '/aruco_udp_start', self.start_callback, 10)
        self.get_logger().info("✅ ArUco UDP 런처 노드 활성화됨")

    def id_callback(self, msg):
        self.target_id = msg.data
        self.get_logger().info(f"[ROS2] 🎯 수신된 target_id: {self.target_id}")

    def start_callback(self, msg):
        if msg.data.strip() != "start":
            self.get_logger().warn("⚠️ 알 수 없는 명령어입니다.")
            return

        if self.target_id is None:
            self.get_logger().warn("⚠️ 아직 /aruco_target_id 토픽이 수신되지 않았습니다.")
            return

        self.get_logger().info("🚀 Pick & Place 프로세스 시작 중...")

        # subprocess 실행
        subprocess.Popen(["python3", "/home/jetcobot/dev_ws/src/mycobot_udp_control/mycobot_udp_control/udp_pick_place_controller.py", str(self.target_id)])

def main(args=None):
    rclpy.init(args=args)
    node = ArucoUDPLauncher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
