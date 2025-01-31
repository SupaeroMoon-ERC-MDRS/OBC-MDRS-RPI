import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64, Bool  # Message type for angle input
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# from adafruit_pca9685 import PCA9685
# from board import SCL, SDA
# import busio
# import argparse
from adafruit_servokit import ServoKit


class GripperControlNode(Node):
    def __init__(self):
        super().__init__('gripper_control_node')
        self.subscription = self.create_subscription(
            Bool,
            '/cmd_grip_arm',  # Topic name
            self.angle_callback,
            10  # QoS
        )

        self.gripped = False
        
        self.kit = ServoKit(channels=16)
        self.kit.servo[8].actuation_range = 300
        self.kit.servo[8].set_pulse_width_range(500, 2500)
        
        self.get_logger().info("Gripper Control Node has been started!")



    def angle_callback(self, msg):
        try:
            self.input = msg.data
            if self.input and self.gripped:
                self.release()
            elif self.input and not self.gripped:
                self.grip()

        except ValueError as e:
            self.get_logger().error(str(e))

    def grip(self):  # assuming this joint is the index not the name
        # Ensure angle is within valid range
        self.kit.servo[8].angle = 90# channel is 8 
        self.gripped = True
        self.get_logger().info("Holding on")

    def release(self):
        self.kit.servo[8].angle = 0 # default position 
        self.gripped = False
        self.get_logger().info("Letting go")
        



def main(args=None):
    rclpy.init(args=args)
    node = GripperControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

