import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info('Robot node has started.')

    def cmd_vel_callback(self, msg):
        self.get_logger().info(f'Received cmd_vel: Linear: {msg.linear.x}, Angular: {msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
