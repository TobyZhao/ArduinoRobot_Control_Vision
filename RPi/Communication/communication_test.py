#!/usr/bin/env python
import sys
import RPi.GPIO as GPIO
import serial
import time
from collections import Counter
import math
import arduinoCom
import os

ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 1)


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

time.sleep(2)

#Try connecting to Ardunio from any of the USB ports
for i in range (10):
    if arduino.connect() == 1:
        print 'connection established'
        break
    else:
        print 'Connection failed'

#Sample code for using the set get protocol
while True:
    cmd = input("Give an input in following format: linear_velocity, angular_velocity, time_in_seconds: ")
    cmd_time = time.time()
    print cmd[0]
    print cmd[1]

    while (time.time()-cmd_time)<cmd[2]:
        arduino.set_value(cmd[0],cmd[1])
        time.sleep(0.1)
