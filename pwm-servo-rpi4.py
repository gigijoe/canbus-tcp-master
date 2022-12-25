# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

#import sys
#from os.path import dirname
#sys.path.append(dirname('/usr/local'))

#
# pip3 install adafruit-circuitpython-motor
# pip3 install adafruit-circuitpython-pca9685
#

import time

from board import SCL, SDA
import busio

# Import the PCA9685 module. Available in the bundle and here:
#   https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

from threading import Thread 

from servolcm import pwm_t
import lcm

i2c = busio.I2C(SCL, SDA)

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)
# You can optionally provide a finer tuned reference clock speed to improve the accuracy of the
# timing pulses. This calibration will be specific to each board and its environment. See the
# calibration.py example in the PCA9685 driver.
# pca = PCA9685(i2c, reference_clock_speed=25630710)
pca.frequency = 50
# To get the full range of the servo you will likely need to adjust the min_pulse and max_pulse to
# match the stall points of the servo.
# This is an example for the Sub-micro servo: https://www.adafruit.com/product/2201
# servo7 = servo.Servo(pca.channels[7], min_pulse=580, max_pulse=2350)
# This is an example for the Micro Servo - High Powered, High Torque Metal Gear:
#   https://www.adafruit.com/product/2307
# servo7 = servo.Servo(pca.channels[7], min_pulse=500, max_pulse=2600)
# This is an example for the Standard servo - TowerPro SG-5010 - 5010:
#   https://www.adafruit.com/product/155
# servo7 = servo.Servo(pca.channels[7], min_pulse=400, max_pulse=2400)
# This is an example for the Analog Feedback Servo: https://www.adafruit.com/product/1404
# servo7 = servo.Servo(pca.channels[7], min_pulse=600, max_pulse=2500)
# This is an example for the Micro servo - TowerPro SG-92R: https://www.adafruit.com/product/169
# servo7 = servo.Servo(pca.channels[7], min_pulse=500, max_pulse=2400)

# The pulse range is 750 - 2250 by default. This range typically gives 135 degrees of
# range, but the default is to use 180 degrees. You can specify the expected range if you wish:
# servo7 = servo.Servo(pca.channels[7], actuation_range=135)

def pwm_handler(channel, data):
	msg = pwm_t.decode(data)
	print("Received message on channel \"%s\" %d %d" % (channel, msg.channel, msg.angle))

	if msg.angle > 180:
		msg.angle = 180
	if msg.angle < 0:
		msg.angle = 0

	#pwm.setRotationAngle(msg.channel, msg.angle)
	servo.Servo(pca.channels[msg.channel]).angle = msg.angle

if __name__ == "__main__":
	#pwm.setServoPulse(1,500) 
	for i in range(16):
		#pwm.setRotationAngle(i, 90)
		servo.Servo(pca.channels[i], actuation_range=180, min_pulse=750, max_pulse=2250).angle = 0

	lc = lcm.LCM()

	s_status = lc.subscribe("PWM", pwm_handler)

	try:
		while True:
			lc.handle()
	except KeyboardInterrupt:
		pass
