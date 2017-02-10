# ArduinoWheeledRobot_Control_Vision

step 1: assemble the hardware(Ardunio UNO, L239D) together into a wheeled robot

step 2: apply PID to control speeds of both wheels (low level control)

step 3: detect circles using Hough transform and filter out false circles

step 4: compute the velocities of wheels based on the radius and center position of the circle detected

step 4: Respberry-Pi transmits motion commands to Arduino (high level control)
