import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import subprocess

class ArucoUDPLauncher(Node):
    def __init__(self):
        super().__init__('aruco_udp_launcher')
        # 단일 토픽 구독: /robot_arm/user_cmd (Int32)
        self.sub_cmd = self.create_subscription(
            Int32, '/robot_arm/user_cmd', self.user_cmd_callback, 10
        )
        self.get_logger().info("ArUco UDP 런처 노드 활성화됨 (/robot_arm/user_cmd 구독)")

    def user_cmd_callback(self, msg: Int32):
        target_id = msg.data
        self.get_logger().info(f"명령 수신: target_id={target_id} → Pick & Place 시작")
        subprocess.Popen([
            "python3",
            "/home/jetcobot/dev_ws/src/mycobot_udp_control/mycobot_udp_control/udp_pick_place_controller.py",
            str(target_id)
        ])

def main(args=None):
    rclpy.init(args=args)
    node = ArucoUDPLauncher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()