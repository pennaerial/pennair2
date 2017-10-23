class Gimbal:
	#assuming 3 servos input
	def __init__(self, s_x, s_y, s_za):
		self.servo_x = s_x
		self.servo_y = s_y
		self.servo_z = s_z

	#set servo to rad change, unless hit limit, then
	#reset to center because UAV made intentionally turn

	#todo after get info on servo class
	#z axis
	def rotateGimbalZ(self, rad):
		pass

	#x axis
	def rotateGimbalX(self, rad):
		pass

	#y axis
	def rotateGimbalY(self, rad):
		pass

	'''
	loop to automatically keep looking in same direction
	unless direction changed by 1 of functions above.

	angular is msgs.angular from a listener, from listerner
	using "from geometry_msgs import Twist"
	'''
	def autoTrackingGimbal(self, angular):
		#if changed, counteract change
		if angular.x != 0:
			rotateGimbalX(x)

		if angular.y != 0:
			rotateGimbalY(y)

		if angular.z != 0:
			rotateGimbalZ(z)




