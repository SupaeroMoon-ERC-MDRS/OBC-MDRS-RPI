import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import roboclaw_driver as roboclaw

class RoboclawCommandNode(Node):
    def __init__(self):
        super().__init__("roboclaw_command_node")

        # Initialize parameters
        self.address = self.declare_parameter("roboclaw_address", 128).get_parameter_value().integer_value
        self.dev_name = self.declare_parameter("device", "/dev/ttyUSB0").get_parameter_value().string_value
        self.baud_rate = self.declare_parameter("baud_rate", 115200).get_parameter_value().integer_value
        self.ticks_per_meter = self.declare_parameter("ticks_per_meter", 4342.2).get_parameter_value().double_value

        # Initialize Roboclaw driver
        self.robo = roboclaw.Roboclaw(self.dev_name, self.baud_rate)
        try:
            self.robo.Open()
            self.get_logger().info("Connected to Roboclaw on " + self.dev_name)
        except Exception as e:
            self.get_logger().error("Could not connect to Roboclaw: " + str(e))
            raise e

        # Subscribe to wheel commands from /wheel_controller/commands
        self.subscription = self.create_subscription(
            Float64MultiArray, "/wheel_controller/commands", self.wheel_command_callback, 10
        )

    def wheel_command_callback(self, msg):
        """ Processes wheel velocity commands and sends to Roboclaw """
        if len(msg.data) != 6:
            self.get_logger().warn("Received incorrect number of wheel commands. Expected 6 values.")
            return

        left_speed = msg.data[0]  # All left wheels have the same speed
        right_speed = msg.data[1]  # All right wheels have the same speed

        # Convert to Roboclaw-compatible speed (ticks/sec)
        left_ticks = int(left_speed * self.ticks_per_meter)
        right_ticks = int(right_speed * self.ticks_per_meter)

        # Send speed commands to Roboclaw
        try:
            self.robo.SpeedM1M2(self.address, right_ticks, left_ticks)
            self.get_logger().info(f"Sent wheel speeds: Right = {right_ticks}, Left = {left_ticks}")
        except Exception as e:
            self.get_logger().error("Error sending speed commands: " + str(e))

def main(args=None):
    rclpy.init(args=args)
    node = RoboclawCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
