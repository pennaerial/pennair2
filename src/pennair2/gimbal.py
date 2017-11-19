import numpy as np
import quaternion as qt
import math
#from servo import Servo

class Gimbal:
	#assuming 3 servos input, when testing don't run servos
	def __init__(self, a, b, c, distance, testmode):
		self.servo_xy = a
		self.servo_yz = b
		self.servo_zx = c
		#distance of gimbal below the UAV
		self.distance = -distance
		self.testmode = testmode

	#expect input of vector
	#set gimbal to point in direction of vector
	def gimbal_set_vector(self, vector):
		#angle in xy plane from x axis towards y axis
		xy_angle = math.acos(vector[0]/np.linalg.norm([vector[0], vector[1], 0]))
		if math.isnan(xy_angle):
			xy_angle = 0

		#angle in yz plane from y axis towards z axis
		yz_angle = math.acos(vector[1]/np.linalg.norm([0, vector[1], vector[2]]))
		if math.isnan(yz_angle):
			yz_angle = 0

		#angle in xz plane form z axis towards x axis
		zx_angle = math.acos(vector[2]/np.linalg.norm([vector[0], vector[1], 0]))
		if math.isnan(zx_angle):
			zx_angle = 0

		if not self.testmode:
			self.servo_x.set_servo_angle(x_angle)
			self.servo_y.set_servo_angle(y_angle)
			self.servo_z.set_servo_angle(z_angle)
		return (xy_angle, yz_angle, zx_angle)


	#input of 2 points, made out of list [x, y, z]
	#set gimbal to point in that direction
	def gimbal_point_in_space(self, UAV, point, quaternion):
		#vector from UAV to point
		vector = [0, 0, 0]
		vector[0] = point[0] - UAV[0]
		vector[1] = point[1] - UAV[1]
		vector[2] = point[2] - UAV[2]
		#vector from UAV to gimbal, use quaternion in case UAV at angle
		dist = [0, 0, self.distance]
		if (quaternion is not None):
			dist = quaternion.rotate(dist)
		#vector from gimbal to point
		final = [vector[0] - dist[0], vector[1] - dist[1], vector[2] - dist[2]]
		return self.gimbal_set_vector(final)

	#return state of 3 servos in a tuple
	def gimbal_get_state(self):
		state = (0, 0, 0)
		if not self.testmode:
			state[0] = self.servo_x.get_servo_angle
			state[1] = self.servo_y.get_servo_angle
			state[2] = self.servo_y.get_servo_angle
		return state
		

