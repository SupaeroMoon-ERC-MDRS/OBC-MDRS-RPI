import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import roboclaw_driver as roboclaw


class RoboclawNode(Node):
    def __init__(self):
        super().__init__("roboclaw_command_node")

        # Initialize parameters
        self.addresses = [int(128), int(129), int(130)]
        self.dev_name = "/dev/ttyAMA0"
        self.baud_rate = 115200
        self.ticks_per_meter = 4342.2

        self.nShutdownAttempts = 10

        # Initialize Roboclaw driver
        self.robo = roboclaw.Roboclaw(self.dev_name, self.baud_rate)
        self.get_logger().info("Starting motor drives")
        try:
            self.robo.Open()
            self.get_logger().info("Connected to Roboclaw on " + self.dev_name)
        except Exception as e:
            self.get_logger().error("Could not connect to Roboclaw: " + str(e))
            raise

        # Subscribe to wheel commands from /wheel_controller/commands
        self.subscription = self.create_subscription(
            Float64MultiArray,
            "/wheel_controller/commands",
            self.wheel_command_callback,
            10,
        )

    def wheel_command_callback(self, msg):
        """Processes wheel velocity commands and sends to Roboclaw"""
        if len(msg.data) != 6:
            self.get_logger().warning(
                "Received incorrect number of wheel commands. Expected 6 values, received {len(msg.data)}."
            )
            return

        left_speed = msg.data[0]  # All left wheels have the same speed
        right_speed = msg.data[1]  # All right wheels have the same speed

        # Convert to Roboclaw-compatible speed (ticks/sec)
        left_ticks = int(left_speed * self.ticks_per_meter)
        right_ticks = int(right_speed * self.ticks_per_meter)

        # Send speed commands to Roboclaw
        for claw in self.addresses:
            try:
                self.robo.SpeedM1M2(claw, right_ticks, left_ticks)
                self.get_logger().info(
                    f"Sent wheel speeds to {claw}: Right = {right_ticks}, Left = {left_ticks}"
                )
            except Exception as e:
                self.get_logger().error(
                    f"Error sending speed commands to {claw}: {str(e)}"
                )
                self.shutdown()

    # need clean shutdown so motors stop even if new msgs are arriving
    def shutdown(self):
        self.get_logger().info("Shutting down")

        nPriorAttempts = 0
        self.shutdownRecursive(nPriorAttempts)

    def shutdownRecursive(self, nPriorAttempts):
        if nPriorAttempts > self.nShutdownAttempts:
            self.get_logger().error("Could not shutdown motors!!!")

        try:
            self.shutDownAddresses()
        except OSError as e:
            self.get_logger().info(
                f"Shutdown did not work trying again (Error: {str(e)})"
            )
            self.shutdownRecursive(nPriorAttempts + 1)

    def shutDownAddresses(self):
        for address in self.addresses:
            self.robo.ForwardM1(address, 0)
            self.robo.ForwardM2(address, 0)


def main(args=None):
    rclpy.init(args=args)
    node = RoboclawNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
