import time, board, busio, adafruit_ina260
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c, address = 0x43)
ina260 = adafruit_ina260.INA260(i2c, address=0x40)

pca.frequency = 50
servo0 = servo.Servo(pca.channels[0])
servo1 = servo.Servo(pca.channels[1])
servo2 = servo.Servo(pca.channels[2])
servo3 = servo.Servo(pca.channels[3])

while True:
	print(ina260.voltage)
	servo0.angle = 0
	servo1.angle = 0
	#servo2.angle = 0
	#servo3.angle = 0
	time.sleep(0.5)
	servo0.angle = 180
	servo1.angle = 180
	#servo2.angle = 180
	#servo3.angle = 180
	time.sleep(0.5)
