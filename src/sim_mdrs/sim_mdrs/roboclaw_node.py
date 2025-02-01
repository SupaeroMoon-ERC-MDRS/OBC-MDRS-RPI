## To create a ROS node to process incoming messages from the remote control
"""Untested code for now"""
import rclpy
from rclpy.node import Node
from roboclaw_driver import Roboclaw

from std_msgs.msg import Float64MultiArray
import numpy as np
import time


class Struct:
    def __init__(self, **kwargs):
        for key, value in kwargs.items():
            setattr(self, key, value)


class msg:
    def __init__(
        self,
        linear_x=0.0,
        linear_y=0.0,
        linear_z=0.0,
        angular_x=0.0,
        angular_y=0.0,
        angular_z=0.0,
    ):
        self.linear = Struct(x=linear_x, y=linear_y, z=linear_z)
        self.angular = Struct(x=angular_x, y=angular_y, z=angular_z)

    def __repr__(self):
        return (
            f"msg(linear: x={self.linear.x}, y={self.linear.y}, z={self.linear.z}, "
            f"angular: x={self.angular.x}, y={self.angular.y}, z={self.angular.z})"
        )


class EncoderOdom:
    def __init__(self, ticks_per_meter, base_width):
        self.TICKS_PER_METER = ticks_per_meter
        self.BASE_WIDTH = base_width
        self.cur_x = 0
        self.cur_y = 0
        self.cur_theta = 0.0
        self.last_enc_left = 0
        self.last_enc_right = 0
        self.last_enc_time = time.time()

    @staticmethod
    def normalize_angle(angle):
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle

    def update(self, enc_left, enc_right):
        left_ticks = enc_left - self.last_enc_left
        right_ticks = enc_right - self.last_enc_right
        self.last_enc_left = enc_left
        self.last_enc_right = enc_right

        dist_left = left_ticks / self.TICKS_PER_METER
        dist_right = right_ticks / self.TICKS_PER_METER
        dist = (dist_right + dist_left) / 2.0

        current_time = time.time()
        d_time = current_time - self.last_enc_time
        self.last_enc_time = current_time

        # TODO find better what to determine going straight, this means slight deviation is accounted
        if left_ticks == right_ticks:
            d_theta = 0.0
            self.cur_x += dist * np.cos(self.cur_theta)
            self.cur_y += dist * np.sin(self.cur_theta)
        else:
            d_theta = (dist_right - dist_left) / self.BASE_WIDTH
            r = dist / d_theta
            self.cur_x += r * (
                np.sin(d_theta + self.cur_theta) - np.sin(self.cur_theta)
            )
            self.cur_y -= r * (
                np.cos(d_theta + self.cur_theta) - np.cos(self.cur_theta)
            )
            self.cur_theta = self.normalize_angle(self.cur_theta + d_theta)

        if abs(d_time) < 0.000001:
            vel_x = 0.0
            vel_theta = 0.0
        else:
            vel_x = dist / d_time
            vel_theta = d_theta / d_time

        return vel_x, vel_theta

    def print_state(self, enc_l, enc_r):
        vx, vth = self.update(enc_l, enc_r)
        current_time = time.time()

        print("Beginning of Odometry message")
        print(f"Current Time: {current_time}")

        print(f"position.x = {self.cur_x}")
        print(f"position.y = {self.cur_y}")
        print(f"position.z = {0.0}")

        print(f"msg.linear.x = {vx}")
        print(f"msg.linear.y = {0}")
        print(f"msg.angular.z = {vth}")
        print("\n\n")


class RoboclawNode(Node):
    def __init__(self):
        super().__init__("roboclaw_node")
        self.subscription = self.create_subscription(
            Float64MultiArray, "/wheel_controller/commands", self.cmd_vel_motors, 10
        )

        # logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

        baud_rate = 115200
        dev_name1 = "/dev/ttyAMA0"  # change


        self.robo = Roboclaw(dev_name1, baud_rate)
        self.addresses = [int(128), int(129), int(130)]  # change
        self.gear_ratio = 26.9
        self.wheel_radius = 0.04
        self.ticks_per_rev = 752
        self.conversion_factor = 1/50
        self.accel = int(16383/2)

        print("Starting motor drives")

        try:
            self.robo.Open()
            self.get_logger().info("Successfully opened serial communications")
        except Exception as e:
            self.get_logger().error("Could not connect to Roboclaw: %s", e)
            raise e
        

        for address in self.addresses:
            try:
                self.get_logger().info(f"Attempting to talk to motor controller {address} through serial port {dev_name1} at a {baud_rate} baud_rate.")
                version = self.robo.ReadVersion(address)
                self.get_logger().info(f"response for RC at {address}: {version}")
                if version[0]:
                    self.get_logger().info(f"Roboclaw Version: {repr(version[1])}")
                else:
                    self.get_logger().warn("Could not get version from Roboclaw")
            except Exception as e:
                self.get_logger().error("Could not connect to Roboclaw: %s", e)
                raise e
            self.robo.ForwardM1(address, 0)
            self.robo.ForwardM2(address, 0)
            # self.robo.ResetEncoders(address)

        self.MAX_SPEED = 2.0  # to be tested
        # self.TICKS_PER_METER = 4342.2  # to be tested
        self.BASE_WIDTH = 0.315  # to be checked
        self.last_set_speed_time = time.time()

        # self.encodm = [
        #     EncoderOdom(self.TICKS_PER_METER, self.BASE_WIDTH),
        #     EncoderOdom(self.TICKS_PER_METER, self.BASE_WIDTH),
        #     EncoderOdom(self.TICKS_PER_METER, self.BASE_WIDTH),
        # ]

        self.get_logger().info("Roboclaw Node Initialized")

    def cmd_vel_motors(self, msg):
        """Handle velocity commands for multiple differential drive motors"""
        try:
            for address in self.addresses:
                left_speed = msg.data[0] #these indexes can also just be 0 and 1 and it should work
                right_speed = msg.data[1]

                # Ticks conversion
                qppsm1 = self.vel_to_qpps(right_speed)
                qppsm2 = self.vel_to_qpps(left_speed)
                self.robo.DutyAccelM1M2(address, self.accel, qppsm1, self.accel, qppsm2)

                # Update timestamp
                self.last_set_speed_time = time.time()

        except Exception as e:
            self.get_logger().error(f"Motor command failed: {str(e)}")
            self.shutdown()

    def vel_to_qpps(self, vel):
        return int(vel * self.gear_ratio * self.conversion_factor * self.ticks_per_rev / (2 * np.pi * self.wheel_radius)) 
    # TODO: need clean shutdown so motors stop even if new msgs are arriving
    def shutdown(self):
        self.get_logger().info("Shutting down")
        try:
            for address in self.addresses:
                self.robo.ForwardM1(address, 0)
                self.robo.ForwardM2(address, 0)
        except OSError:
            self.get_logger().info("Shutdown did not work trying again")
            try:
                for address in self.addresses:
                    self.robo.ForwardM1(address, 0)
                    self.robo.ForwardM2(address, 0)
            except OSError as e:
                self.get_logger().error("Could not shutdown motors!!!!")
                self.get_logger().error(e)


def main(args=None):
    rclpy.init(args=args)
    node = RoboclawNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
