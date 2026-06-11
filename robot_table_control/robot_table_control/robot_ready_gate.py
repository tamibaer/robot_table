import rclpy
from rclpy.node import Node
from ur_dashboard_msgs.srv import Load


class RobotReadyGate(Node):
    def __init__(self):
        super().__init__('robot_ready_gate')
        self.client = self.create_client(Load, '/dashboard_client/load_program')
        self.timer = self.create_timer(5.0, self.check_service)
        self.get_logger().info("Waiting for /dashboard_client/load_program service...")

    def check_service(self):
        if self.client.service_is_ready():
            self.get_logger().info("Dashboard service is ready → triggering external control")
            self.timer.cancel()
            rclpy.shutdown()


def main():
    rclpy.init()
    node = RobotReadyGate()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()
