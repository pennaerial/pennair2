import cv2
import numpy as np
import shapeify
from shapedetector import ShapeDetector
from hsv import ColorsHSV

def stratify(name, bounds, ignore=None):
	# Read image
	frame = cv2.imread(name, cv2.IMREAD_COLOR)
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	lower = None
	upper = None
	mask = None

	if bounds == "all":
		masks = []
		colors = ["white", "black", "grey", "red", 
		"blue", "green", "yellow", "purple", "brown",
		"orange"]

		if ignore is not None:
			colors = [x for x in colors if x not in ignore]

		h, w, _ = frame.shape
		combined = np.zeros((h, w), np.uint8)
		for col in colors:
			l, u = getBounds(col)
			if l is not None:
				temp = cv2.inRange(hsv, l, u)
				masks.append((temp, col))
		for m in masks:
			cv2.bitwise_or(combined, m[0], combined)

		mask = combined
	else:	
		# set bounds of what you want to keep
		sensitivity = 10
		lower, upper = getBounds(bounds)
		# make mask
		mask = cv2.inRange(hsv, lower, upper)

	get_color = np.zeros((mask.shape[0], mask.shape[1], 3), np.uint8)
	for x in range(mask.shape[0]):
		for y in range(mask.shape[1]):
			if mask[x][y] >= 1:
				get_color[x][y][0] = 255
				get_color[x][y][1] = 255
				get_color[x][y][2] = 255
			else:
				get_color[x][y][0] = 0
				get_color[x][y][1] = 0
				get_color[x][y][2] = 0

	cv2.fastNlMeansDenoisingColored(src=get_color, dst=get_color, h=75)
	shapeify.shapeify3D(get_color, frame)


# input: string of color name
# output: lower and upper bounds for that color
def getBounds(bounds):
	lower = None
	upper = None
	if bounds == "white":
		lower, upper = ColorsHSV.white
	
	elif bounds == "black":
		lower, upper = ColorsHSV.black
		
	elif bounds == "grey":
		lower, upper = ColorsHSV.grey

	elif bounds == "red":
		lower, upper = ColorsHSV.red

	elif bounds == "blue":
		lower, upper = ColorsHSV.blue

	elif bounds == "green":
		lower, upper = ColorsHSV.green

	elif bounds == "yellow":
		lower, upper = ColorsHSV.yellow

	elif bounds == "purple":
		lower, upper = ColorsHSV.purple

	elif bounds == "brown":
		lower, upper = ColorsHSV.brown

	elif bounds == "orange":
		lower, upper = ColorsHSV.orange

	else:
		print("Invalid color")
	return lower, upper