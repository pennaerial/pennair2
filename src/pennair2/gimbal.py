import numpy as np
import quaternion as qt
import math
#import servo

class Gimbal:
	#assuming 3 servos input
	def __init__(self, s_x, s_y, s_z, distance):
		self.servo_x = s_x
		self.servo_y = s_y
		self.servo_z = s_z
		#distance of gimbal below the UAV
		self.distance = distance

	#expect input of vector
	#set gimbal to point in direction of vector
	def gimbal_set_vector(self, vector):
		xy_plane = [1, 1, 0]
		yz_plane = [0, 1, 1]
		xz_plane = [1, 0, 1]
		x_angle = math.acos(np.dot(yz_plane, vector)/(np.linalg.norm(vector)*np.linalg.norm(yz_plane)))
		y_angle = math.acos(np.dot(xz_plane, vector)/(np.linalg.norm(vector)*np.linalg.norm(xz_plane)))
		z_angle = math.acos(np.dot(xy_plane, vector)/(np.linalg.norm(vector)*np.linalg.norm(xy_plane)))
		self.servo_x.set_servo_angle(x_angle)
		self.servo_y.set_servo_angle(y_angle)
		self.servo_z.set_servo_angle(z_angle)
		print([x_angle, y_angle, z_angle])


	#input of 2 points, made out of list [x, y, z]
	#set gimbal to point in that direction
	def gimbal_point_in_space(self, UAV, point, quaternion):
		vector = [0, 0, 0]
		vector[0] = point[0] - UAV[0]
		vector[1] = point[1] - UAV[1]
		vector[2] = point[2] - UAV[2]
		vector = qt.multiply(vector, quaternion)
		gimbal_set_vector(vector)

	#return state of 3 servos in a list
	def gimbal_get_state(self):
		state = [0, 0, 0]
		state[0] = self.servo_x.get_servo_angle
		state[1] = self.servo_y.get_servo_angle
		state[2] = self.servo_y.get_servo_angle
		return state
		
g = Gimbal(None, None, None, 0)
g.gimbal_set_vector([2 , 3, 1])


