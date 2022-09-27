# import os
import sys
import time 
import signal
import threading
import curses # key

from threading import Thread 
from PCA9685 import PCA9685

from servolcm import pwm_t
import lcm

pwm = PCA9685()

def lcm_task():
	try:
		while True:
			lc.handle()
	except KeyboardInterrupt:
		pass

def pwm_handler(channel, data):
	msg = pwm_t.decode(data)
	print("Received message on channel \"%s\" %d %d" % (channel, msg.channel, msg.angle))

	pwm.setRotationAngle(msg.channel, msg.angle)

if __name__ == "__main__":
	
	pwm.setPWMFreq(50)
	#pwm.setServoPulse(1,500) 
	for i in range(16):
		pwm.setRotationAngle(i, 90)

	lc = lcm.LCM()

	s_status = lc.subscribe("PWM", pwm_handler)

	lcm_thread = Thread(target=lcm_task)
	lcm_thread.daemon = True
	lcm_thread.start()

	while True:
		time.sleep(100)
