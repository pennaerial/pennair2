# import the necessary packages
import cv2
import math

# todo: implement ratios between peri and other vars
# for more complex shapes
class ShapeDetector:
	def __init__(self):
		pass

	def detect(self, c):
		# initialize the shape name and approximate the contour
		shape = "unidentified"
		peri = cv2.arcLength(c, True)
		approx = cv2.approxPolyDP(c, 0.03 * peri, True)

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

			# a square will have an aspect ratio that is approximately
			# equal to one, otherwise, the shape is a rectangle
			shape = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"

			#todo trapezoid

		# if the shape is a pentagon, it will have 5 vertices
		elif len(approx) == 5:
			is_pent = self.similar_sides(approx)
			if is_pent[0]:
				shape = "pentagon"
			else:
				checking = self.check_circular(is_pent[1], peri)
				if checking == 1:
					shape = "half circle"
				else:
					shape = "quarter circle"

		elif len(approx) == 6:
			is_hex = self.similar_sides(approx)
			if is_hex[0]:
				shape = "hexagon"
			else:
				checking = self.check_circular(is_hex[1], peri)
				if checking == 1:
					shape = "half circle"
				else:
					shape = "quarter circle"
				
		elif len(approx) == 7:
			shape = "heptagon"

		elif len(approx) == 8:
			shape = "octagon"

		elif len(approx) == 10 or len(approx) == 9:
			shape = "star"

		elif len(approx) == 12 or len(approx) == 11 or len(approx) == 13 or len(approx) == 14:
			shape = "cross"

		# otherwise, we assume the shape is a circle
		else:
			shape = "circle"

		# return the name of the shape
		return shape


	def get_distance(self, one, two):
		return math.sqrt(pow((one[0][0] - two[0][0]), 2) + pow((one[0][1] - two[0][1]), 2))


	def similar_sides(self, approx):
		side_lengths = []
		is_similar = True
		for elem in range(len(approx)-1):
			side_lengths.append(self.get_distance(approx[elem], approx[elem + 1]))
		side_lengths.append(self.get_distance(approx[0], approx[len(approx) - 1]))
		avg = sum(side_lengths)/len(side_lengths)
		for side in side_lengths:
			if not (side >= avg*0.7 and side <= avg*1.3):
				is_similar = False
		return [is_similar, side_lengths]


	# 1 for semicircle, 2 for quarter circle
	def check_circular(self, side_lengths, peri):
		diameter = max(side_lengths)
		expected_peri = math.pi*diameter/2 + diameter
		if peri >= expected_peri*0.8 and peri <= expected_peri*1.2:
			return 1
		else:
			return 2
