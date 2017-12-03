from servo import Servo
from math import pi
from time import sleep


if __name__=="__main__":

	seconds = 1

	print("make servo")
	test = Servo(50, 0, pi, pi/2)
	sleep(seconds)

	print("rotation 1")
	test.set_servo_angle(pi/4)
	sleep(seconds)

	print("rotation 2")
	test.set_servo_angle(3*pi/2)
	sleep(seconds)

	print("rotation 3")
	test.set_servo_angle(0)
	sleep(seconds)

	print("rotation 4")
	test.set_servo_angle(pi)
	sleep(seconds)

	print("rotation 5")
	test.set_servo_angle(pi+1)

	test = Servo(1, 0, pi, pi/2)
	sleep(3)
	test.set_servo_angle(pi/4)
	sleep(3)
	test.set_servo_angle(3*pi/2)


