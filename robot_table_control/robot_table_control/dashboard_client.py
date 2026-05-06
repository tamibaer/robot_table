import rclpy
from rclpy.node import Node

from ur_dashboard_msgs.srv import Load
from std_srvs.srv import Trigger


class DashboardClient(Node):

    def __init__(self):
        super().__init__('dashboard_client')

        self.load_client = self.create_client(
            Load,
            '/dashboard_client/load_program'
        )

        self.play_client = self.create_client(
            Trigger,
            '/dashboard_client/play'
        )

        self.get_logger().info("Waiting for UR dashboard services...")

        if not self.load_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("Load service not available")

        if not self.play_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("Play service not available")

        self.get_logger().info("Services are ready.")

    def load_program(self, filename: str) -> bool:
        req = Load.Request()
        req.filename = filename

        self.get_logger().info(f"Loading program: {filename}")

        future = self.load_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        resp = future.result()

        if not resp or not resp.success:
            self.get_logger().error("Load failed")
            return False

        self.get_logger().info("Program loaded successfully.")
        return True

    def play(self) -> bool:
        req = Trigger.Request()

        self.get_logger().info("Starting program...")

        future = self.play_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        resp = future.result()

        if not resp or not resp.success:
            self.get_logger().error("Play failed")
            return False

        self.get_logger().info("Program started successfully.")
        return True


def main(args=None):
    rclpy.init(args=args)

    node = DashboardClient()

    try:
        program_name = "external_control.urp"

        if not node.load_program(program_name):
            node.get_logger().error("Aborting: load failed")
            return

        if not node.play():
            node.get_logger().error("Aborting: play failed")
            return

        node.get_logger().info("External Control is running. Shutting down node.")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()