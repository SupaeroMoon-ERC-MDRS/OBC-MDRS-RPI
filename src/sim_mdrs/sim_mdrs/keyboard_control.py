import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

# Instructions for users
instructions = """
Reading from the keyboard and publishing to /cmd_vel!
---------------------------
Moving around:
   w
a  s  d
   x

w/x : increase/decrease linear speed
a/d : increase/decrease angular speed
s/space : stop
CTRL-C to quit
"""

# Key bindings
move_bindings = {
    'w': (1, 0),
    'x': (-1, 0),
    'a': (0, 1),
    'd': (0, -1),
}

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.linear_speed = 0.0  # Initial linear speed
        self.angular_speed = 0.0  # Initial angular speed

        self.linear_increment = 0.1  # Speed increment for linear motion
        self.angular_increment = 0.2  # Speed increment for angular motion

        self.max_linear_speed = 5.0  # Maximum linear speed
        self.max_angular_speed = 3.0  # Maximum angular speed

        print(instructions)
        self.settings = termios.tcgetattr(sys.stdin)
        self.timer = self.create_timer(0.1, self.run)


    def get_key(self):
        """Read a single keypress from the keyboard."""
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key


    def run(self):
        key = self.get_key()
        print(f"Current Speed: Linear = {self.linear_speed}, Angular = {self.angular_speed}")
        if key in move_bindings:
            linear_dir, angular_dir = move_bindings[key]

            # Increment the speed in the direction of the key press
            self.linear_speed = linear_dir * (self.linear_speed + self.linear_increment)
            self.angular_speed = angular_dir * (self.angular_speed + self.angular_increment)

            # Clamp the speeds to their maximum values
            self.linear_speed = max(min(self.linear_speed, self.max_linear_speed), -self.max_linear_speed)
            self.angular_speed = max(min(self.angular_speed, self.max_angular_speed), -self.max_angular_speed)

            twist = Twist()
            twist.linear.x = self.linear_speed
            twist.angular.z = self.angular_speed
            self.publisher.publish(twist)
            
        elif key == ' ' or key == 's':
            self.linear_speed = 0.0
            self.angular_speed = 0.0
            twist = Twist()
            self.publisher.publish(twist)

        elif key == '\x03':  # CTRL-C
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    keyboard_control = TeleopKeyboard()
    
    try:
        rclpy.spin(keyboard_control)
    except KeyboardInterrupt:
        pass
    finally:
        keyboard_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
