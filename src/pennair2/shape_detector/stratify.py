import cv2
import numpy as np
import shapeify

# inputs: name of file, string of color name
# output: none
def stratify(name, bounds):
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
		h, w, _ = frame.shape
		combined = np.zeros((h, w), np.uint8)
		for col in colors:
			l, u = getBounds(col)
			if l is not None:
				temp = cv2.inRange(hsv, l, u)
				masks.append(temp)
		for m in masks:
			cv2.bitwise_or(combined, m, combined)

		mask = combined
		#cv2.fastNlMeansDenoising(src=mask, dst=mask, h=75)
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

	cv2.fastNlMeansDenoisingColored(src=get_color, dst=get_color, h=60)
	shapeify.shapeify3D(get_color)


# input: string of color name
# output: lower and upper bounds for that color
def getBounds(bounds):
	lower = None
	upper = None
	if bounds == "white":
		pass
	elif bounds == "black":
		pass
	elif bounds == "gray":
		pass
	elif bounds == "red":
		lower = np.array([0, 100, 100])
		upper = np.array([10, 255, 255])

	elif bounds == "blue":
		lower = np.array([100,150,0])
		upper = np.array([140,255,255])

	elif bounds == "green":
		pass

	elif bounds == "yellow":
		lower = np.array([20,100,100])
		upper = np.array([30,255,255])

	elif bounds == "purple":
		pass

	elif bounds == "brown":
		pass

	elif bounds == "orange":
		lower = np.array([10, 100, 20])
		upper = np.array([25, 255, 255])

	else:
		print("Invalid color")
	return lower, upper