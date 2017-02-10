#!/usr/bin/env python
import sys
import RPi.GPIO as GPIO
import serial
import time
from collections import Counter
import math
import arduinoCom
import os
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import time
import cv2


def get_distance(x1, y1, x2, y2):
    return (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)

#Variable to enable or disable code on Pi, this variable can be triggered from switch later.
Run = True
#Pin configuration type BCM
GPIO.setmode(GPIO.BCM)

#Close Serial Connections to Raspberry Pi
def Close_connection():
    GPIO.cleanup()
    robot.closeSerial()
    sys.exit('Empty Sear Found - Task finished')

#Create an Arduino Object which has base code for communication between Pi and Arduino
arduino = arduinoCom.Arduino()
ser= serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)

#Try connecting to Ardunio from any of the USB ports
for i in range (10):
    if arduino.connect() == 1:
        print 'connection established'
        break
    else:
        print 'Connection failed'


# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
res_x = 160
res_y = 120
camera.resolution = (res_x, res_y)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(160, 120))

camera.awb_mode = "off"
camera.awb_gains = (1.1, 2.3)

# set up hsv range
lower_green = np.array([44, 59, 15])
upper_green = np.array([99, 255, 255])

# allow the camera to warmup
time.sleep(0.1)

stear = 0
radius = 0
x_old = 0
y_old = 0
stop = True
w1 = 0
w2 = 0
        
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    image = frame.array
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    radius = None
    
    circles = cv2.HoughCircles(gray_image, cv2.HOUGH_GRADIENT, dp=10, minDist = 50)
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv_image, lower_green, upper_green)
        res = cv2.bitwise_and(image, image, mask=mask)
        
        circles_2 = []
        for (x, y, r) in circles:
            cv2.circle(gray_image, (x,y), r,(255,0,0))
            if res[y][x][0] > 10:
                cv2.circle(gray_image, (x,y), r,(255,0,0), thickness=5)
                circles_2.append((x, y, r))
                
        best_match_index = 0
        index = 0
        min_dist = 10000000
        
        # filter false circles
        if len(circles_2) > 1:
            for (x, y, r) in circles_2:
                dist = get_distance(x, y, x_old, y_old)
                if dist < min_dist:
                    min_dist = dist
                    best_match_index = index
                index += 1
        if len(circles_2) > 0:
            (x, y, r) = circles_2[best_match_index]
            
            # correct the radius
            true_r = 0
            test_x = x
            test_y = y
            while res[test_y][x][0] > 10 and res[y][test_x][0] > 10:
                true_r += 1
                test_x = x + true_r
                test_y = y + true_r
                if test_x >= res_y:
                    break
                if test_y >= res_x:
                    break
            radius = int(r*0.6 + true_r*0.4)
            p1 = (x+r, y+r)
            p2 = (x-r, y-r)
            cv2.rectangle(gray_image, p1, p2, (255,0,0), thickness=2)
            stop = False
        else:
            stop = True
        circles = None
        

    cv2.imshow("Frame", gray_image)
    #cv2.imshow("Frame2", res)

    # calculate and print out velocities
    if radius > 40 or radius is None or stop:
        print "The robot has stoped"
        w1 = 0
        w2 = 0
    else:
        v = 0.2
        k = 0.004
        error = float(x - (res_x/2) )
        if abs(error) < 5:
            error = 0
        delta = k* error
        w1 = v - delta
        w2 = v + delta
        print "w1: %f \t w2: %f \t delta: %f \t x: %d \t err: %f" %(w1, w2, delta, x, error)
        
    cmd_time = time.time()
    while (time.time()-cmd_time) < 1:
        arduino.set_value(w2)
        time.sleep(0.05)


    x_old = x
    y_old = y
    """
    if not stear == 0 and radius < 60:
        cmd_time = time.time()
        cmd1 = 2.0 - 0.005 * stear
        cmd2 = 2.0 + 0.005 * stear
        print cmd1, cmd2, r
        while (time.time()-cmd_time) < 1:
            arduino.set_value(cmd1, cmd2)
            time.sleep(0.1)
    elif radius > 70:
        cmd_time = time.time()
        cmd1 = 0
        cmd2 = 0
        print cmd1, cmd2, r
        while (time.time()-cmd_time) < 1:
            arduino.set_value(cmd1, cmd2)
            time.sleep(0.1)
    """
    # show the frame
    #cv2.imshow("Frame", gray_image)
    
    key = cv2.waitKey(1) & 0xFF

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
    
