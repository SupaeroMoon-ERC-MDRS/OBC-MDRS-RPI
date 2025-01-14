import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from tf_transformations import quaternion_from_euler
import math

ROVER_WHEEL_RADIUS = 0.075
TRACK_WIDTH = 0.253  # Equivalent to d4 in your C++ code

class DifferentialDriveController(Node):
    def __init__(self):
        super().__init__('controller')
        self.motor_wheel_pub = self.create_publisher(Float64MultiArray, '/wheel_controller/commands', 1)
        self.odom_pub = self.create_publisher(Odometry, 'osr/odom', 10)
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 1)
        
        self.x_position = 0.0
        self.y_position = 0.0
        self.theta = 0.0
        self.left_vel = 0.0
        self.right_vel = 0.0
        self.last_time = self.get_clock().now()

    def cmd_vel_callback(self, msg):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        self.left_vel = (linear_velocity - (angular_velocity * TRACK_WIDTH / 2)) / ROVER_WHEEL_RADIUS
        self.right_vel = (linear_velocity + (angular_velocity * TRACK_WIDTH / 2)) / ROVER_WHEEL_RADIUS

        self.publish_wheel_commands()
        self.update_odometry()

    def publish_wheel_commands(self):
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [self.left_vel, self.right_vel, self.left_vel, self.right_vel, self.left_vel, self.right_vel]
        self.motor_wheel_pub.publish(wheel_msg)

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        linear_velocity = (self.left_vel + self.right_vel) * ROVER_WHEEL_RADIUS / 2
        angular_velocity = (self.right_vel - self.left_vel) * ROVER_WHEEL_RADIUS / TRACK_WIDTH

        self.x_position += linear_velocity * math.cos(self.theta) * dt
        self.y_position += linear_velocity * math.sin(self.theta) * dt
        self.theta += angular_velocity * dt

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        odom_msg.pose.pose.position.x = self.x_position
        odom_msg.pose.pose.position.y = self.y_position

        quat = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DifferentialDriveController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
