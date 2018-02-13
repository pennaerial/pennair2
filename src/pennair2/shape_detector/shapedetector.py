# import the necessary packages
import cv2
import math
import numpy as np
import imutils

# todo: implement ratios between peri and other vars
# for more complex shapes
class ShapeDetector:
	def __init__(self):
		pass

	def detect(self, c):
		# initialize the shape name and approximate the contour
		shape = "unidentified"
		peri = cv2.arcLength(c, True)
		approx = cv2.approxPolyDP(c, 0.04 * peri, True)
		area = cv2.contourArea(c)
		rect=cv2.minAreaRect(c)
		box = cv2.boxPoints(rect)
		box = np.int0(box)
		side_l = self.get_distance_simple(box[0], box[1])
		side_w = self.get_distance_simple(box[1], box[2])
		min_box_area = side_l*side_w

		# if the shape is a triangle, it will have 3 vertices
		if len(approx) == 3:
			shape = "triangle"

		# if the shape has 4 vertices, it is either a square or
		# a rectangle
		elif len(approx) == 4:
			# compute the bounding box of the contour and use the
			# bounding box to compute the aspect ratio
			(x, y, w, h) = cv2.boundingRect(approx)
			ar = w / float(h)

			#check angles between sides if trapezoid
			angles = []
			angles.append(self.get_angle(approx[0], approx[1], approx[3]))
			angles.append(self.get_angle(approx[1], approx[0], approx[2]))
			angles.append(self.get_angle(approx[2], approx[1], approx[3]))
			angles.append(self.get_angle(approx[3], approx[0], approx[2]))
			is_sq = self.similar_sides(approx)
			side_lengths = is_sq[1]

			# is rectangle or square
			if min_box_area < area * 1.1 and min_box_area > area * 0.9:
				shape = "rect or sq"
			# else is trapezoid or part of circle
			elif ((np.isclose([angles[0]], [angles[1]], 0.2) and np.isclose([angles[2]], [angles[3]], 0.2))
				or (np.isclose([angles[1]], [angles[2]], 0.2) and np.isclose([angles[3]], [angles[0]], 0.2))):
				shape = "trapezoid"
			else:
				shape = self.check_circular(side_w, side_l, area)

		# if the shape is a pentagon, it will have 5 vertices
		elif len(approx) == 5:
			is_pent = self.similar_sides(approx)
			if is_pent[0]:
				shape = "pentagon"
			else:
				shape = self.check_circular(side_w, side_l, area)

		elif len(approx) == 6:
			is_hex = self.similar_sides(approx)
			if is_hex[0]:
				shape = "hexagon"
			else:
				shape = self.check_circular(side_w, side_l, area)
				
		elif len(approx) == 7:
			shape = "heptagon"

			check = self.check_cross(side_w, side_l, area)
			if check:
				shape = "cross"

		elif len(approx) == 8:
			shape = "octagon"

			check = self.check_cross(side_w, side_l, area)
			if check:
				shape = "cross"

		elif len(approx) == 10 or len(approx) == 9:
			shape = "star"

		elif len(approx) == 12 or len(approx) == 11 or len(approx) == 13 or len(approx) == 14:
			shape = "cross"

		# otherwise, we assume the shape is a circle
		else:
			shape = "circle"

		#check circle because circle has issues
		if side_w < side_l*1.03 and side_w > side_l*0.97 and len(approx) >= 6:
			radius = side_w/2
			expected_area = pow(radius,2)*math.pi
			if area < expected_area*1.03 and area > expected_area*0.97:
				shape="circle"

		# return the name of the shape
		return shape

	def get_distance_approx(self, one, two):
		return math.sqrt(pow((one[0][0] - two[0][0]), 2) + pow((one[0][1] - two[0][1]), 2))

	def get_distance_simple(self, one, two):
		return math.sqrt(pow((one[0] - two[0]), 2) + pow((one[1] - two[1]), 2))

	# tail of both vector 1 and 2 at center
	def get_angle(self, center, one, two):
		vector1 = [center[0][0]-one[0][0],center[0][1]-one[0][1]]
		vector2 = [center[0][0]-two[0][0],center[0][1]-two[0][1]]
		top = vector1[0]*vector2[0]+vector1[1]*vector2[1]
		bottom = self.get_distance_approx(center, one) * self.get_distance_approx(center, two)
		return np.arccos(top/bottom)

	def similar_sides(self, approx):
		side_lengths = []
		is_similar = True
		for elem in range(len(approx)-1):
			side_lengths.append(self.get_distance_approx(approx[elem], approx[elem + 1]))
		side_lengths.append(self.get_distance_approx(approx[0], approx[len(approx) - 1]))
		avg = sum(side_lengths)/len(side_lengths)
		for side in side_lengths:
			if not (side >= avg*0.7 and side <= avg*1.3):
				is_similar = False
		return [is_similar, side_lengths]

	def check_cross(self, side_w, side_l, area):
		side = max(side_w, side_l)
		near_circle = pow(side/2, 2) * math.pi
		if abs(area - (side_w * side_l)/2) < abs(area - near_circle):
			return True
		else:
			return False

	# 1 for semicircle, 2 for quarter circle
	def check_circular(self, side_w, side_l, area):
		side = max(side_w, side_l)
		expected_qt = pow(side, 2) * math.pi/4
		expected_half = pow(side/2, 2) * math.pi/2
		expected_full = pow(side/2, 2) * math.pi
		diff_qt = abs(area - expected_qt)
		diff_half = abs(area - expected_half)
		diff_full = abs(area - expected_full)
		min_diff = min(diff_qt, min(diff_half, diff_full))

		if min_diff == diff_qt:
			return "quarter circle"
		elif min_diff == diff_half:
			return "half circle"
		else: 
			return "circle"
