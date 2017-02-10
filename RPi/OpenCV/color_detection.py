# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import numpy as np
import cv2
res = [320, 240]
center = (res[0]/2,res[1]/2)

M = cv2.getRotationMatrix2D(center, 180, 1.0)

def nothing(x):
    pass

# Create a black image, a window
# img = np.zeros((res[0],res[1],3), np.uint8)
cv2.namedWindow('image')

# create trackbars for color change
cv2.createTrackbar('H_low','image',70,255,nothing)
cv2.createTrackbar('S_low','image',0,255,nothing)
cv2.createTrackbar('V_low','image',0,255,nothing)
cv2.createTrackbar('H_high','image',95,255,nothing)
cv2.createTrackbar('S_high','image',255,255,nothing)
cv2.createTrackbar('V_high','image',255,255,nothing)

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (res[0], res[1])
camera.framerate = 32
camera.awb_mode = 'off'
rg, bg = (1.1, 2.3)
camera.awb_gains = (rg, bg)
rawCapture = PiRGBArray(camera, size=(res[0], res[1]))


# allow the camera to warmup
time.sleep(0.1)

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    # img = cv2.warpAffine(frame.array,M,(res[0],res[1]))
    img = frame.array
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # get current positions of four trackbars
    H_low = cv2.getTrackbarPos('H_low','image')
    S_low = cv2.getTrackbarPos('S_low','image')
    V_low = cv2.getTrackbarPos('V_low','image')
    H_high = cv2.getTrackbarPos('H_high','image')
    S_high = cv2.getTrackbarPos('S_high','image')
    V_high = cv2.getTrackbarPos('V_high','image')
    lower = np.array([H_low,S_low,V_low])
    upper = np.array([H_high,S_high,V_high])
    mask = cv2.inRange(hsv_img, lower, upper)
    masked_img = cv2.bitwise_and(img,img,mask=mask)

    cv2.imshow("frame1", masked_img)
    cv2.imshow("frame2", img)

    key = cv2.waitKey(1) & 0xFF
	# clear the stream in preparation for the next frame
    rawCapture.truncate(0)
	# if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
