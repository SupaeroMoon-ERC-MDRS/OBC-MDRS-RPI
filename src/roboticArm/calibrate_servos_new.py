"""
Simple calibration/test script for PCA9685 PWM motor control

Moves motors back and forth

Setup:
$ pip3 install adafruit-circuitpython-servokit

Instructions: https://learn.adafruit.com/adafruit-16-channel-servo-driver-with-raspberry-pi/using-the-adafruit-library
"""
from time import sleep
import time
import argparse
import numpy as np

from adafruit_servokit import ServoKit

ang_vel = 360  # degrees per second

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
                        prog='ServoCalibrator',
                        description='Helps calibrate the corner motors. When running this script, make sure that the servo motors can move freely.',
                        epilog='If you need help, please ask on Slack on the #troubleshooting channel!')
    
    parser.add_argument('joint', type=int, choices=range(16), help="Which channel on the PCA the motor is connected to")
    # parser.add_argument('angle_vel', type=int, help="angle between 120 and 180, vel between -1 and 1.")
    parser.add_argument('target_angle', type=int, help="target angle to reach")
    parser.add_argument('curr_angle',type = int, help = "starting angle of motor (if possible)")
    args = parser.parse_args()

    kit = ServoKit(channels=16)
    sleep(0.1)

    try: 
        ang_diff = args.target_angle - args.curr_angle
        dt = abs(ang_diff/ang_vel)
        kit.continuous_servo[args.joint].throttle = np.sign(ang_diff)*0.3
        sleep(dt)
        kit.continuous_servo[args.joint].throttle = 0

        print(f"Servo motor at channel {args.joint} was set to {args.target_angle}")
    except:
        kit.continuous_servo[args.joint].throttle = 0
        raise
