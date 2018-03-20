# import the necessary packages
from shapedetector import ShapeDetector
import process_alpha as alpha
import argparse
import imutils
import cv2
import sys
import numpy as np
import math

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
                cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
                cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 0, 255), 2)

                cropped = crop_image(c, color_image)
                if cropped is not None:
                    # show the output image
                    # cv2.imshow("Image", image)
                    cv2.imshow("Crop", cropped)
                    cv2.waitKey(0)
        except ZeroDivisionError:
            pass

#cut out smallest square that fits around a contour
def crop_image(c, image): 
    x,y,w,h = cv2.boundingRect(c)
    longest = max(w, h)
    cropped = None
    paddingAbove = None
    paddingLeft = None
    # need to vertically center shape
    if longest == w:
        center = math.floor(y+h/2)
        left = center-math.floor(w/2)
        right = left + w
        if left < 0:
            left = 0
        if right >= image.shape[0]:
            right = image.shape[0]
        paddingAbove = left
        paddingLeft = x
        cropped = image[left:right, x:x+w, 0::]
    # need to horizontally center shape
    else:
        center = math.floor(x+w/2)
        left = center-math.floor(h/2)
        right = left + h
        if left < 0:
            left = 0
        if right >= image.shape[1]:
            right = image.shape[1]
        paddingAbove = y
        paddingLeft = left
        cropped = image[y:y+h, left:right, 0::]

    cX, cY = alpha.get_center(c)
    alpha.stratify_color(cropped, c, cX - paddingLeft, cY - paddingAbove)
    return cropped

def extract_letter(c, image):

    pass

if __name__=="__main__":
    shapeify3D(sys.argv[1])

            