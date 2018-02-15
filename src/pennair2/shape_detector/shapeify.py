# import the necessary packages
from shapedetector import ShapeDetector
import argparse
import imutils
import cv2
import sys
import numpy as np

# given picture, call other function to label picture
def shapeify3D(image, color_image):

    resized = imutils.resize(image, width=300)
    ratio = image.shape[0] / float(resized.shape[0])
    image_area = image.shape[0] * image.shape[1]

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
    sd = ShapeDetector()

    # loop over the contours
    for c in cnts:
        try:
            area = cv2.contourArea(c)
            if area > image_area*0.0001:
                # compute the center of the contour, then detect the name of the
                # shape using only the contour
                M = cv2.moments(c)
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
                    0.5, (0, 0, 255), 2)

                #orient(c, image)

                # show the output image
                cv2.imshow("Image", image)
                cv2.waitKey(0)
        except ZeroDivisionError:
            pass

#testing add-ons that might work for orientation
def orient(c, image): 
    try:
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        # line of box
        line_r, line_c =image.shape[:2]
        [vx, vy, x, y] = cv2.fitLine(box, cv2.DIST_L2,0,0.01,0.01)
        if vx < 0.01: vx = 0.0000001
        lefty = int((-x*vy/vx) + y)
        righty = int(((image.shape[1]-x)*vy/vx)+y)
        cv2.line(image,(image.shape[1]-1,righty),(0,lefty),(255,255,0),2)
        # line of item
        line_r, line_c =image.shape[:2]
        [vx, vy, x, y] = cv2.fitLine(c, cv2.DIST_L2,0,0.01,0.01)
        if vx < 0.01: vx = 0.0000001
        lefty = int((-x*vy/vx) + y)
        righty = int(((image.shape[1]-x)*vy/vx)+y)
        cv2.line(image,(image.shape[1]-1,righty),(0,lefty),(255,0,0),2)
    except Exception:
        pass

if __name__=="__main__":
    shapeify3D(sys.argv[1])

            