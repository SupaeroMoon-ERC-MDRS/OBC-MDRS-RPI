import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist, Quaternion
import sys
import termios
import tty
import numpy as np


class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.publisher_ = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.subscriber_ = self.create_subscription(Quaternion, '/cmd_move_arm', self.listener_callback, 10)

        # Initialize joint positions
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]  # ['joint2', 'joint3', 'joint4', 'joint5']
        self.angular_increment = 0.1  # Increment step for joint positions
        self.linear_increment = 0.5  # Increment step for joint positions

        self.l_arm_inf = 22.8399
        self.l_arm_sup = 26.5

        self.arm_pos = [0.0, 0.0, self.l_arm_inf + self.l_arm_sup, 0.0] # [theta, x, y, gamma]
        self.arm_cmd = [0, 0, 0, 0]

    def listener_callback(self, msg):
        """
        Callback function called when a message is received.
        """
        self.get_logger().info(f"Received message: '{[msg.y, msg.x, msg.z, msg.w]}'")
        self.arm_cmd = [msg.y, msg.x, msg.z, msg.w]
        self.control_loop()

    def send_command(self):
        msg = JointTrajectory()
        msg.joint_names = ['joint2', 'joint3', 'joint4', 'joint5']

        point = JointTrajectoryPoint()
        point.positions = self.joint_positions.copy()
        point.time_from_start.sec = 2
        point.time_from_start.nanosec = 0

        msg.points.append(point)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Command sent: {self.joint_positions}')

    def update_arm_position(self, ind, direction):
        self.prev_pos = self.arm_pos #store last position in case new position outside limits
        self.arm_pos[ind] += direction * self.linear_increment
        if self.check_pos_limits():
        ## Check if arm_pos is within sphere of motion
            self.cart_to_joints()
            self.send_command()
        else:
            self.arm_pos = self.prev_pos

    def update_arm_angular(self, ind, direction):
        self.prev_pos = self.arm_pos
        self.arm_pos[ind] += direction * self.angular_increment
        if self.check_pos_limits():
            self.joint_positions[ind] = self.arm_pos[ind]
            self.send_command()
        else:
            self.arm_pos = self.prev_pos

    def check_pos_limits(self):
        ## Set up for upright when all joints are 90
        r_max = self.l_arm_inf + self.l_arm_sup
        r_min = np.sqrt(self.l_arm_inf**2 + self.l_arm_sup**2)
        r_pos = np.sqrt(self.arm_pos[1]**2 + self.arm_pos[2]**2)
        y_min = -self.l_arm_sup
        x_min = self.l_arm_sup - self.l_arm_inf
        theta_lim = 1*np.pi
        if r_pos > r_max: #checking if target is too far
            print("Man's reach exceeds his grasp")
            #self.get_logger().info("Target position too far")
            return False
        elif self.arm_pos[1] < x_min: #close target forces t3 to go beyond limit #this limit will change to xmin if we change the setup
            print("I need some more personal space")
            #self.get_logger().info("Target position too close")
            return False
        elif self.arm_pos[2] < y_min: #too low means t2 goes over 180
            print("Aim a little higher")
            #self.get_logger().info("Target position too low")
            return False
        elif self.arm_pos[0] < 0 or self.arm_pos[0] > theta_lim: #need to check how much base motor can swivel
            print("Don't look back in anger")
            #self.get_logger().info("Target exceeds base motor rotation limit")
            return False
        elif self.arm_pos[3] < 0 or self.arm_pos[3] > np.pi: #NEEDS REVIEW - this is for t4 angle limit
            print("Ow-ouch! Stop that, my wrist hurts")
            #self.get_logger().info("Target beyond gripper rotation limit")
            return False
        else:
            return True


    def control_loop(self):
        try:
            if not all(x == 0 for x in self.arm_cmd):
                for joint_index, direction in enumerate(self.arm_cmd):
                    if joint_index in [1, 2]:
                        self.update_arm_position(joint_index, direction)
                    else:
                        self.update_arm_angular(joint_index, direction)

        except KeyboardInterrupt:
            self.get_logger().info('Keyboard control stopped.')

    def cart_to_joints(self):
        # Base angle
        t1 = self.arm_pos[0]  # [theta, x, y, gamma]
        px = self.arm_pos[1]
        py = self.arm_pos[2]
        gamma = self.arm_pos[3]

        l1 = self.l_arm_inf
        l2 = self.l_arm_sup

        self.get_logger().info(f"Current Position:\n theta: {t1}\n px: {px}\n py: {py}\n gamma: {gamma}\n")

        f = (px**2 + py**2 - l1**2 - l2**2)/(2*l1*l2)
        self.get_logger().info(f"Current Position:\n L1: {l1}\n L2: {l2}\n F: {f}\n")
        g = np.sqrt(l2**2 - f**2)

        q3_1 = -np.arctan2(l2,0) + np.arctan2(f, g)
        q3_2 = -np.arctan2(l2,0) + np.arctan2(f, -g)

        t3 = max(q3_1, q3_2)

        m = l1 + l2*np.cos(t3)
        n = l2*np.sin(t3)

        c_q2 = (px*m + py*n)/(m**2 + n**2)

        s_q2 = (py - n*c_q2)/m

        t2 = np.arctan2(s_q2, c_q2)

        # Solve last joint angle
        t4 = gamma - t2 - t3

        self.joint_positions = [t1, t2, t3, t4]



def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    try:
        rclpy.spin(node)  # Keep the node running
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()  # Clean up resources
        rclpy.shutdown()     # Shutdown ROS 2


if __name__ == '__main__':
    main()
