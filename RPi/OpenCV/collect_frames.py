
# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import argparse

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (320, 240)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(320, 240))

camera.awb_mode = 'off'
camera.awb_gains = ( 1.2, 1.8)

#set hsv range
low_green = np.array([61, 92, 0])
high_green = np.array([72, 78, 0])
##high_green = np.array([94, 255, 255])



# allow the camera to warmup
time.sleep(0.1)

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    image = frame.array
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # detect circles in the image
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.5, 100)
 
    # ensure at least some circles were found
    if circles is not None:
	# convert the (x, y) coordinates a.nd radius of the circles to integers
	circles = np.round(circles[0, :]).astype("int")
	
         # detect color
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv_image, low_green, high_green)
        res = cv2.bitwise_and(image, image, mask=mask)
        
# loop over the (x, y) coordinates and radius of the circles
	for (x, y, r) in circles:
		# draw the circle in the output image, then draw a rectangle
		# corresponding to the center of the circle
            if not res[y][x][0] == 0:
		cv2.circle(gray, (x, y), r, (0, 255, 0))

 

    # show the frame
    cv2.imshow("Frame", gray)
    key = cv2.waitKey(1) & 0xFF

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
