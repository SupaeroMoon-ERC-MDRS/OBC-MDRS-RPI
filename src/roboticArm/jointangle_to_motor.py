##Modified code
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32  # Message type for angle input
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# from adafruit_pca9685 import PCA9685
# from board import SCL, SDA
# import busio
# import argparse
from adafruit_servokit import ServoKit

# import RPi.GPIO as GPIO  # Imports the standard Raspberry Pi GPIO library
from time import sleep  # Imports sleep (aka wait or pause) into the program

# GPIO.setmode(GPIO.BOARD) # Sets the pin numbering system to use the physical layout


class MotorControlNode(Node):
    def __init__(self):
        super().__init__("motor_control_node")

        self.kit = ServoKit(channels=16)

        # Define mapping of joints to PCA9685 channels
        self.joint_channels = {
            "joint2": 0,
            "joint3": 1,
            "joint4": 2,
            "joint5": 3,
        }  # will need to ensure this index  mapping matches the channels - else alter code a bit
        self.arm_curr_pos = [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]  # 1st is imaginary, and there's no gripper here #this should
        """Check if some channel mapping is required to identify the joints"""
        sleep(0.1)

        # Initialising some characterisitcs of the motors
        self.servo_motors = [3]
        self.cont_motors = [1, 2]
        self.cont_ang_vel = 360  # deg/s #how do we know this?

        for idx in self.servo_motors:  # to set angle limits on servo motors
            # check if these intialisation values are correct
            self.kit.servo[idx].actuation_range = 300
            self.kit.servo[idx].set_pulse_width_range(500, 2500)

        for idx in self.cont_motors:
            ## apparently it's optional to set the pulse width range for the continuous servo
            # self.kit.continuous_servo[idx].set_pulse_width_range(500, 2500)
            # important thing is to set them all to zero to begin with
            self.kit.continuous_servo[
                idx
            ].throttle = 0  # throttle values go from -1.0 to 1.0

        self.subscription = self.create_subscription(
            JointTrajectory,
            "/arm_controller/joint_trajectory",  # Topic name
            self.angle_callback,
            10,  # QoS
        )
        self.get_logger().info(
            'MotorControlNode started. Listening for angle input on "motor_angle" topic.'
        )

    def angle_callback(self, msg):
        """
        Callback for the motor_angle topic. Receives the angle and runs the motor.
        """

        # Extract trajectory points
        for joint, point in zip(msg.joint_names, msg.points):
            angle = point.positions
            try:
                self.run_motor(joint, angle)
                self.get_logger().info(
                    f"Received angle: {angle}° and applied PWM signal."
                )
            except ValueError as e:
                self.get_logger().error(str(e))

    def run_motor(self, joint, angle):  # assuming this joint is the index not the name
        """
        Convert an angle to PWM and run the motor on the specified channel.
        """
        # In case input joint is the name and not the index, get index from mapping
        joint = self.joint_channels[joint]

        # Ensure angle is within valid range
        if angle < 0 or angle > 180:  # may not be required
            raise ValueError("Angle must be between 0 and 180 degrees.")

        if joint in self.servo_motors:
            self.kit.servo[joint].angle = angle
            self.arm_curr_pos[joint] = angle
        elif joint in self.cont_motors:
            ang_diff = angle - self.arm_curr_pos[joint]
            dt = ang_diff / self.cont_ang_vel
            self.kit.continuous_servo[joint].throttle = np.sign(ang_diff)
            sleep(dt)
            self.kit.continuous_servo[joint].throttle = 0
            self.arm_curr_pos[joint] = angle

        # self.set_angle(self.joint_pins[joint], angle)

        # Convert angle to PWM signal
        # min_pulse = 1000  # Min pulse length in microseconds
        # max_pulse = 2000  # Max pulse length in microseconds
        # pulse_range = max_pulse - min_pulse
        # pulse_length = min_pulse + (angle / 180.0) * pulse_range
        # pwm_value = int(pulse_length / 1000000 * self.pca.frequency * 4096)

    def destroy(self):
        """
        Cleanup resources.
        """
        self.pca.deinit()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down.")
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
