import cv2
import numpy as np
import copy
import math
import time
from picamera.array import PiRGBArray
from picamera import PiCamera
import sys
import RPi.GPIO as GPIO
import serial
from collections import Counter
import os
import arduinoCom


def removeBG(img):
    fgmask = bgModel.apply(img)
    kernel = np.ones((3, 3), np.uint8)
    fgmask = cv2.erode(fgmask, kernel, iterations = 1)
    res = cv2.bitwise_and(img, img, mask = fgmask)

    return res

def fingernumber(cnt, drawing):
    hull = cv2.convexHull(cnt, returnPoints = False)
    if len(hull) > 3:
        defects = cv2.convexityDefects(cnt, hull)
    count_defects = 0
    cv2.drawContours(thresh1, contours, -1, (0, 255, 0), 3)
    for i in range(defects.shape[0]):
        s, e, f, d = defects[i, 0]
        start = tuple(cnt[s][0])
        end = tuple(cnt[e][0])
        far = tuple(cnt[f][0])
        a = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
        b = math.sqrt((far[0] - start[0])**2 + (far[1] - start[1])**2)
        c = math.sqrt((end[0] - far[0])**2 + (end[1] - far[1])**2)
        angle = math.acos((b**2 + c**2 - a**2)/(2*b*c)) * 57
        if angle <= 90:
            count_defects += 1
            cv2.circle(crop_img, far, 1, [0, 0, 255], -1)
        #dist = cv2.pointPolygonTest(cnt,far,True)
        cv2.line(crop_img, start, end, [0, 255, 0], 2)
#cv2.circle(crop_img,far,5,[0,0,255],-1)
    return count_defects

#Close Serial Connections to Raspberry Pi
def Close_connection():
    GPIO.cleanup()
    robot.closeSerial()
    sys.exit('Empty Sear Found - Task finished')

def speed(count_defects):
    v = 0.15
    delta = 0.05
    if count_defects == 2 :
        w2 = v
        cv2.putText(img, "forward!", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
    elif count_defects == 3:
        w2 = v + delta
        cv2.putText(img, "turn left!", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
    elif count_defects == 5 or count_defects == 4:
        w2 = v - delta
        cv2.putText(img, "turn right!", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
    else :
        w2 = 0
        cv2.putText(img, "Stop!", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)

    return w2
    

#Variable to enable or disable code on Pi, this variable can be triggered from switch later.
Run = True
#Pin configuration type BCM
GPIO.setmode(GPIO.BCM)

#Create an Arduino Object which has base code for communication between Pi and Arduino
arduino = arduinoCom.Arduino()
ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 1)
time.sleep(2)

#Try connecting to Ardunio from any of the USB ports
for i in range (10):
    if arduino.connect() == 1:
        print 'connection established'
        break
    else:
        print 'Connection failed'
        
##main program

camera = PiCamera()
res_x = 260
res_y = 170
#res_x = 160
#res_y = 120
half_x = int(res_x/2)
half_y = int(res_y/2)
camera.resolution = (res_x, res_y)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size = (res_x, res_y))
x = int(res_x * 0.4 * 0.5)
y = int(res_y * 0.75)

time.sleep(0.1)

for frame in camera.capture_continuous(rawCapture, format = "bgr", use_video_port = True):

    img = frame.array

    M = cv2.getRotationMatrix2D((half_x, half_y), 180, 1)
    img = cv2.warpAffine(img, M, (res_x, res_y))
    

    bgModel = cv2.createBackgroundSubtractorMOG2(detectShadows = True)

    ##draw rectangle in the original image
    img = cv2.flip(img, 1)
    cv2.rectangle(img, (half_x - x, 20), (half_x + x, y), (0, 255, 0), 0)  
    crop_img = img[20 : y, (half_x - x) : (half_x + x)]
    gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
    value = (41, 41)
    blurred = cv2.GaussianBlur(gray, value, 0)
    _, thresh1 = cv2.threshold(blurred, 130, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    ##  cv2.imshow('Thresholded', thresh1)
    
    crop_img = removeBG(crop_img)

##check the version of python
    (version, _, _) = cv2.__version__.split('.')

    if version is '3':
        ##find the contour, NONE stores all contour pixels
        image, contours, hierarchy = cv2.findContours(thresh1.copy(), \
               cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    elif version is '2':
        contours, hierarchy = cv2.findContours(thresh1.copy(),cv2.RETR_TREE, \
               cv2.CHAIN_APPROX_NONE)

    cnt = max(contours, key = lambda x: cv2.contourArea(x))

    hull = cv2.convexHull(cnt)
    drawing = np.zeros(crop_img.shape, np.uint8)
    cv2.drawContours(drawing, [cnt], 0, (0, 255, 0), 2)
    cv2.drawContours(drawing, [hull], 0, (0, 0, 255), 3)

## find convexity defects
    count_defects = fingernumber(cnt, drawing)
## calculate the speed
    w2 = speed(count_defects)

    cmd_time = time.time()
    while (time.time() - cmd_time) < 1:
        #arduino.set_value(w1, w2)
        arduino.set_value(w2)
        time.sleep(0.05)

    #cv2.imshow('drawing', drawing)
    #cv2.imshow('end', crop_img)
    cv2.imshow('Gesture', img)
    all_img = np.hstack((drawing, crop_img))
    cv2.imshow('Contours', all_img)
##keyboard
    key = cv2.waitKey(1) & 0xFF

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break  
