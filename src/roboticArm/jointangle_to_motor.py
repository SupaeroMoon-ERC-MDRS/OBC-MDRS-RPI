import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32  # Message type for angle input
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio
import RPi.GPIO as GPIO  # Imports the standard Raspberry Pi GPIO library
from time import sleep   # Imports sleep (aka wait or pause) into the program
GPIO.setmode(GPIO.BOARD) # Sets the pin numbering system to use the physical layout


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        self.joint_pins = {'joint2': 10, 'joint3': 11, 'joint4': 12, 'joint5': 13} # pinout of GPIOs

        for joint, pin in self.joint_pins:
            GPIO.setup(pin,GPIO.OUT)
            self.pin = pin
            self.p = GPIO.PWM(pin, 50)
            self.p.start(0) 

        self.subscription = self.create_subscription(
            Float32,
            '/arm_controller/joint_trajectory',  # Topic name
            self.angle_callback,
            10  # QoS
        )
        self.get_logger().info('MotorControlNode started. Listening for angle input on "motor_angle" topic.')

    def angle_callback(self, msg):
        """
        Callback for the motor_angle topic. Receives the angle and runs the motor.
        """
        
        # Extract trajectory points
        for joint, point in zip(msg.joint_names, msg.points):
            angle = point.positions
            try:
                self.run_motor(joint, angle)
                self.get_logger().info(f'Received angle: {angle}° and applied PWM signal.')
            except ValueError as e:
                self.get_logger().error(str(e))

    def run_motor(self, joint, angle):
        """
        Convert an angle to PWM and run the motor on the specified channel.
        """
        # Ensure angle is within valid range
        if angle < 0 or angle > 180:
            raise ValueError("Angle must be between 0 and 180 degrees.")
        
        self.set_angle(self.joint_pins[joint], angle)

        # Convert angle to PWM signal
        # min_pulse = 1000  # Min pulse length in microseconds
        # max_pulse = 2000  # Max pulse length in microseconds
        # pulse_range = max_pulse - min_pulse
        # pulse_length = min_pulse + (angle / 180.0) * pulse_range
        # pwm_value = int(pulse_length / 1000000 * self.pca.frequency * 4096)


    def set_angle(self, pin, angle):
        duty = angle / 18 + 2
        GPIO.output(pin, True)
        self.p.ChangeDutyCycle(duty)
        sleep(1)
        GPIO.output(pin, False)
        self.p.ChangeDutyCycle(0) # remove this if motor goes to zero after every angle

    def destroy(self):
        """
        Cleanup resources.
        """
        self.pca.deinit()
        super().destroy_node()
        self.p.stop()
        GPIO.cleanup()

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
