import RPi.GPIO as GPIO  # Imports the standard Raspberry Pi GPIO library
from time import sleep   # Imports sleep (aka wait or pause) into the program
GPIO.setmode(GPIO.BOARD) # Sets the pin numbering system to use the physical layout

class Servo():
    def __init__(self, pin):
        GPIO.setup(pin,GPIO.OUT)  # Sets up pin 11 to an output (instead of an input)
        self.pin = pin
        self.p = GPIO.PWM(pin, 50)     # Sets up pin 11 as a PWM pin at 50Hz
        self.p.start(0)               # Starts running PWM on the pin and sets it to 0


    def SetAngle(self, angle):
        duty = angle / 18 + 2
        GPIO.output(self.pin, True)
        self.p.ChangeDutyCycle(duty)
        sleep(1)
        GPIO.output(self.pin, False)
        self.p.ChangeDutyCycle(0)

    def stop(self):
        self.p.stop()                 # At the end of the program, stop the PWM
        GPIO.cleanup()           # Resets the GPIO pins back to defaults

    def calibrate(self):
        while True:
            a_d = input("Test Mode [a/d]: ")
            if a_d == 'a':
                test_angle = input("Angle to test: ")
                self.SetAngle(test_angle)
            elif a_d == 'd':
                test_dc = input("Duty cycle to test: ")
                GPIO.output(3, True)
                self.p.ChangeDutyCycle(test_dc)
                sleep(1)
                GPIO.output(3, False)
                self.p.ChangeDutyCycle(0)
            else:
                print("Please provide a valid input.")


if __name__ == "__main__":
    pin = 4
    servo = Servo(pin)
    try:
        servo.calibrate()
    except KeyboardInterrupt:
        print("Closing connection...")
    except Exception as e:
        raise(e)
    finally:
        servo.stop()


"""
The amount of time the signal is on sets the angle the servo motor will rotate to. 
In most servos, the expected frequency is 50Hz, or 3000 cycles per minute. 
Servos will set to 0 degrees if given a signal of .5 ms, 90 when given 1.5 ms, 
and 180 when given 2.5ms pulses. This translates to about 2.5-12.5% duty in a 50Hz PWM cycle.
"""