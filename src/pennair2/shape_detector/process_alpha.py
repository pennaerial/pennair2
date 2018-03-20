import stratify
import cv2

def stratify_color(matrix, c, x, y):
	found = False;
	colors = ["white", "black", "grey", "red", 
		"blue", "green", "yellow", "purple", "brown",
		"orange"]
	print(str(x) + ", " + str(y))
	for col in colors:
		l, u = stratify.getBounds(col)
		mask = cv2.inRange(matrix, l, u)
		if mask[x][y] == 255:
			print("center is white on color " + col)

def get_center(c):
	M = cv2.moments(c)
	cX = int(M["m10"] / M["m00"])
	cY = int(M["m01"] / M["m00"])
	return cX, cY