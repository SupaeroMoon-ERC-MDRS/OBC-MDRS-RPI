import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32  # Message type for angle input
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        # Initialize I2C and PCA9685
        self.i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = 50  # 50 Hz for servo motors

        # Define motor channel
        self.channel = 0  # Default channel is 0; change if needed

        # Subscribe to the motor_angle topic
        self.subscription = self.create_subscription(
            Float32,
            'motor_angle',  # Topic name
            self.angle_callback,
            10  # QoS
        )
        self.subscription  # Prevent unused variable warning
        self.get_logger().info('MotorControlNode started. Listening for angle input on "motor_angle" topic.')

    def angle_callback(self, msg):
        """
        Callback for the motor_angle topic. Receives the angle and runs the motor.
        """
        angle = msg.data
        try:
            self.run_motor(angle)
            self.get_logger().info(f'Received angle: {angle}° and applied PWM signal.')
        except ValueError as e:
            self.get_logger().error(str(e))

    def run_motor(self, angle):
        """
        Convert an angle to PWM and run the motor on the specified channel.
        """
        # Ensure angle is within valid range
        if angle < 0 or angle > 180:
            raise ValueError("Angle must be between 0 and 180 degrees.")

        # Convert angle to PWM signal
        min_pulse = 1000  # Min pulse length in microseconds
        max_pulse = 2000  # Max pulse length in microseconds
        pulse_range = max_pulse - min_pulse
        pulse_length = min_pulse + (angle / 180.0) * pulse_range
        pwm_value = int(pulse_length / 1000000 * self.pca.frequency * 4096)

        # Set PWM signal on the specified channel
        self.pca.channels[self.channel].duty_cycle = pwm_value

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
        node.get_logger().info('Shutting down.')
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
