#include "motors.h"

const double DEG_PER_TIC = (double)360/(double)384;
const double RAD_PER_DEG = (double)3.141/(double)180;
const double WHEEL_RADIUS = 0.0625/2.0;
const double ROBOT_BASE = 0.201;
const double TICS_PER_ROTATION = 384;
unsigned int left_enc_count, right_enc_count;
unsigned long time_prev, time_now;
double left_angular_vel, right_angular_vel, left_vel = 0, right_vel = 0, line_vel, v = 0.2, delta;
byte motion;    //0 : line, 1 : circle
//Create motor objects with connections and  parameters
//arguements: encoder pin, to motor : out1, out2, enable pin,inverse direction,Kp,Ki,Kd
Motor left_motor(3,8,9,5,false,3.5,2.7,0);
Motor right_motor(2,11,10,6,true,3.5,2.5,0);

void setup() {
  //Serial communication initialization
  Serial.begin(9600);
  
  //Choose motion status
  motion = 0;
  line_vel = 0;

  if (motion == 1){
    vel_compute(1, 0.3, true);
  }
  else{
    left_vel = line_vel;
    right_vel = line_vel;
  }

  //Configure interrupt pins for encoders
  attachInterrupt(digitalPinToInterrupt(left_motor.ENCODER_PIN), left_tic_counter, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_motor.ENCODER_PIN), right_tic_counter, CHANGE);
}

void loop() {
  //function to read and intrepret the serial data received from raspberrypi
  //calculate delta and right velocity
  left_vel = leftreadSerialCmd();
  if (left_vel < 0.01){
    left_vel = 0;
    right_vel = 0;
  }
  else{
    delta = v - left_vel;
    right_vel = v + delta;
  }
  if (left_vel < 0){
    left_vel = 0;
  }
  if (right_vel < 0){
    right_vel = 0;
  }
  
  //move Robot - Use below functions to set reference speed and direction of motor.
  left_motor.rotate(left_vel/WHEEL_RADIUS/(RAD_PER_DEG * DEG_PER_TIC * TICS_PER_ROTATION), 1);
  right_motor.rotate(right_vel/WHEEL_RADIUS/(RAD_PER_DEG * DEG_PER_TIC * TICS_PER_ROTATION), 1);
  left_angular_vel = left_motor.angular_vel;
  right_angular_vel = right_motor.angular_vel;

  //Serial Monitor
//  Serial.print(left_angular_vel * WHEEL_RADIUS * (RAD_PER_DEG * DEG_PER_TIC * TICS_PER_ROTATION));Serial.print(", ");
//  Serial.println(right_angular_vel * WHEEL_RADIUS * (RAD_PER_DEG * DEG_PER_TIC * TICS_PER_ROTATION));
  Serial.print(left_vel);Serial.print(", ");
  Serial.println(right_vel);
   //Serial.print(left_motor.pwm_show); Serial.print(", "); Serial.println(right_motor.pwm_show);

  //Serial Plotter
  //Serial.println(left_angular_vel * WHEEL_RADIUS * (RAD_PER_DEG * DEG_PER_TIC * TICS_PER_ROTATION)); Serial.print(", ");
  //Serial.println(right_angular_vel * WHEEL_RADIUS * (RAD_PER_DEG * DEG_PER_TIC * TICS_PER_ROTATION));
  
  delay(50);  //Defines control loop frequency  
}

//Compute velocity for each wheel when the car preceeds in circle
void vel_compute(double Radius_Circle, double w_Circle, bool CLOCKWISE){
  if (CLOCKWISE == true){
    left_vel = w_Circle * (2.0 * Radius_Circle + ROBOT_BASE / 2.0) / 2.0;
    right_vel = w_Circle * (2.0 * Radius_Circle - ROBOT_BASE / 2.0) / 2.0;
  }
  else{
    left_vel = w_Circle * (2.0 * Radius_Circle - ROBOT_BASE / 2.0) / 2.0;
    right_vel = w_Circle * (2.0 * Radius_Circle + ROBOT_BASE / 2.0) / 2.0;
  }
}

//Callback functions when interrupt are triggered by encoders
void left_tic_counter(){
  //call motor tick counter
  left_motor.tic_counter();
}
void right_tic_counter(){
  //call motor tick counter
  right_motor.tic_counter();
}


