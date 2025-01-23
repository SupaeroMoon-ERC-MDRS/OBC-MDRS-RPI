import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import termios
import tty
import numpy as np

# Key mappings
key_mapping = {
    'q': (0, 1),  # Increase base
    'a': (0, -1), # Decrease base
    'w': (1, 1),  # Increase x
    's': (1, -1), # Decrease x
    'e': (2, 1),  # Increase y
    'd': (2, -1), # Decrease y
    'r': (3, 1),  # Increase grip
    'f': (3, -1)  # Decrease grip
}

class KeyboardArmController(Node):
    def __init__(self):
        super().__init__('keyboard_arm_controller')
        self.publisher_ = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

        # Initialize joint positions
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]  # ['joint2', 'joint3', 'joint4', 'joint5']
        self.angular_increment = 0.1  # Increment step for joint positions
        self.linear_increment = 0.5  # Increment step for joint positions

        self.l_arm_inf = 22.8399
        self.l_arm_sup = 26.5

        self.arm_pos = [0.0, 0.0, self.l_arm_inf + self.l_arm_sup, 0.0] # [theta, x, y, gamma]

        self.get_logger().info('Keyboard controller initialized. Use Q/A W/S E/D R/F keys to move joints. Press Ctrl+C to exit.')

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
        self.arm_pos[ind] += direction * self.linear_increment
        self.cart_to_joints()
        self.send_command()

    def update_arm_angular(self, ind, direction):
        self.arm_pos[ind] += direction * self.angular_increment
        self.joint_positions[ind] = self.arm_pos[ind]
        self.send_command()

    def get_key(self):
        # Capture a single key press from the terminal
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def keyboard_control_loop(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                if key in key_mapping:
                    joint_index, direction = key_mapping[key]
                    if joint_index in [1, 2]:
                        self.update_arm_position(joint_index, direction)
                    else:
                        self.update_arm_angular(joint_index, direction)
                elif key == '\x03':  # Ctrl+C
                    break
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
    node = KeyboardArmController()
    node.keyboard_control_loop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
