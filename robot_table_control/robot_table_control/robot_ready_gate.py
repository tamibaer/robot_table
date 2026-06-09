import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class RobotReadyGate(Node):
    def __init__(self):
        super().__init__('robot_ready_gate')
        self.sub = self.create_subscription(
            Bool,
            '/robot_ready',
            self.cb,
            10
        )
        self.ready = False

    def cb(self, msg):
        print("TESTTESTETTSTTSTSTSTSTSTS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        if msg.data and not self.ready:
            self.get_logger().info("Robot ready received → exiting gate node")
            self.ready = True
            rclpy.shutdown()

def main():
    rclpy.init()
    node = RobotReadyGate()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()