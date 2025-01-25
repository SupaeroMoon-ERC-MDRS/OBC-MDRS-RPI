import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RoboClawNode(Node):
    def __init__(self):
        super().__init__('roboclaw_node')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, world!'
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RoboClawNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()