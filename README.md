# ArduinoRobot-Control-Vision

## project 1: circle tracker

- step 1: assemble the hardware(Ardunio UNO, L239D) together into a wheeled robot

- step 2: apply PID to control speeds of both wheels (low level control)

- step 3: detect circles using Hough transform and filter out false circles

- step 4: compute the velocities of wheels based on the radius and center position of the circle detected

- step 5: Respberry-Pi transmits motion commands to Arduino (high level control)

You can read the script ../RPi/Communication/pi_main.py for more details. [Related instruction](https://vladimirli.gitbooks.io/el2222/content/) 


## project 2: hand gesture control

The basic idea is that you show your hand in front of the camera, and the raspberry pi will pre-process the images (subtract the background using thresholding) gathered from the camera and detect the convex hull enclosing your hand. Different convexity forms different patterns, which can be used for controlling the robot to act differently.

You can read the script ../RPi/communication/main_gesture.py for more details.
