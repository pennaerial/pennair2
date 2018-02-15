# storage for old code in case some of it
# becomes useful so I don't have to rewrite it

# shapedetector.py ------------------------------------------------------------------




# shapeify.py ------------------------------------------------------------------

# input: image from stratify
def shapeify2D(image):
    # load the image and resize it to a smaller factor so that
    # the shapes can be approximated better
    resized = imutils.resize(image, width=300)
    ratio = image.shape[0] / float(resized.shape[0])

    # convert the resized image to resizedscale, blur it slightly,
    # and threshold it
    blurred = cv2.GaussianBlur(resized, (5, 5), 0)
    thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]

    # find contours in the thresholded image and initialize the
    # shape detector
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]
    sd = ShapeDetector()

    # loop over the contours
    for c in cnts:
        # compute the center of the contour, then detect the name of the
        # shape using only the contour
        M = cv2.moments(c)
        try:
            cX = int((M["m10"] / M["m00"]) * ratio)
            cY = int((M["m01"] / M["m00"]) * ratio)

            shape = sd.detect(c)

            # multiply the contour (x, y)-coordinates by the resize ratio,
            # then draw the contours and the name of the shape on the image
            c = c.astype("float")
            c *= ratio
            c = c.astype("int")
            cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
            cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2)

            # show the output image
            cv2.imshow("Image", image)
            cv2.waitKey(0)
        except ZeroDivisionError:
            pass

# returns list of tuples with contour and color of each shape
def returnContoursColor3D(mask, image, string):
    resized = imutils.resize(image, width=300)
    ratio = image.shape[0] / float(resized.shape[0])

    # convert the resized image to grayscale, blur it slightly,
    # and threshold it
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]

    # find contours in the thresholded image and initialize the
    # shape detector
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]
    result = []
    for ct in cnts:
        result.append((ct, string))
    return result

def loopDrawContours(image, contour_list):
    for ct in contour_list:
        c = ct[0]
        try:
            # compute the center of the contour, then detect the name of the
            # shape using only the contour
            M = cv2.moments(c)
            cX = int((M["m10"] / M["m00"]) * ratio)
            cY = int((M["m01"] / M["m00"]) * ratio)
            shape = sd.detect_with_color(c, ct[1])

            # multiply the contour (x, y)-coordinates by the resize ratio,
            # then draw the contours and the name of the shape on the image
            c = c.astype("float")
            c *= ratio
            c = c.astype("int")
            cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
            cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 0, 255), 2)

            #orient(c, image)

            # show the output image
            cv2.imshow("Image", image)
            cv2.waitKey(0)
        except ZeroDivisionError:
            pass

# stratify.py ------------------------------------------------------------------

# input: list of colors
# output: list of tuples of shape contours and colors
def collectShapeTuples(lst, hsv):
	shape_list = []
	for col in lst:
		lower, upper = getBounds(col)
		if lower is not None:
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
			shape_list = shape_list + shapeify.returnContoursColor3D(mask, hsv, col)
	return shape_list

def stratifyContours(name, bounds):
	# Read image
	frame = cv2.imread(name, cv2.IMREAD_COLOR)
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	if bounds == "all":
		colors = ["white", "black", "grey", "red", 
		"blue", "green", "yellow", "purple", "brown",
		"orange"]
		collected = collectShapeTuples(colors, hsv)

		sd = ShapeDetector()
		shapeify.loopDrawContours(hsv, collected)
	else:
		pass

    # inputs: name of file, string of color name
# output: nonedef loopDrawContours(image, contour_list):
    for ct in contour_list:
        c = ct[0]
        try:
            # compute the center of the contour, then detect the name of the
            # shape using only the contour
            M = cv2.moments(c)
            cX = int((M["m10"] / M["m00"]) * ratio)
            cY = int((M["m01"] / M["m00"]) * ratio)
            shape = sd.detect_with_color(c, ct[1])

            # multiply the contour (x, y)-coordinates by the resize ratio,
            # then draw the contours and the name of the shape on the image
            c = c.astype("float")
            c *= ratio
            c = c.astype("int")
            cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
            cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 0, 255), 2)

            #orient(c, image)

            # show the output image
            cv2.imshow("Image", image)
            cv2.waitKey(0)
        except ZeroDivisionError:
            pass
