from servo import Servo
from math import pi
from time import sleep


if __name__=="__main__":
	test = Servo(1, 0, pi, pi/2)
	sleep(3)
	test.set_servo_angle(pi/4)
	sleep(3)
	test.set_servo_angle(3*pi/2)


