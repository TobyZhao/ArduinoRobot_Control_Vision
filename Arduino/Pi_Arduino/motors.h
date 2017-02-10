const double MICRO_PER_SEC = 1000000.0;
//Motor Class with different paramters for configuring the connections
class Motor{
private:
  bool INVERSE;
  double Kp, Ki, Kd, scale = 20;
  unsigned long time_prev, time_now, time_diff;
  double err_prev = 0, err_integ = 0;
  double TICS_PER_ROTATION = 384; //This can be varied based on interrupt configuration
  int current_pwm = 0;
  double thres = 50, deadzone = 0.051;

public:
  byte ENCODER_PIN, OUT1, OUT2, ENB;
  unsigned int tic_count = 0;
  unsigned int tic_count_prev = 0;
  int pwm_show = 0;
  double angular_vel;

  Motor(byte enc_pin, byte out1, byte out2, byte enb, bool inverse,
    double Kp, double Ki, double Kd);
  void calc_angular_vel();
  void rotate(double ref, int direction);
  double get_angular_vel();
  void tic_counter();
  void pid_controller(double ref);
};

//Initialize the motor parameters
Motor::Motor(byte enc_pin, byte out1, byte out2, byte enb, bool inverse,
  double Kp, double Ki, double Kd){
  this->ENCODER_PIN = enc_pin;
  this->OUT1 = out1;
  this->OUT2 = out2;
  this->ENB = enb;
  this->INVERSE = inverse;
  this->Kp = Kp;
  this->Kd = Kd;
  this->Ki = Ki;
  this->time_prev = micros();
  pinMode(enc_pin, INPUT_PULLUP);
  pinMode(out1, OUTPUT);
  pinMode(out2, OUTPUT);
  pinMode(enb, OUTPUT);
}

//function to calculate angular velocity
void Motor::calc_angular_vel(){ 
  time_diff = time_now - time_prev;
  angular_vel = (tic_count - tic_count_prev) * MICRO_PER_SEC / TICS_PER_ROTATION / time_diff;
  delay(50);
}

//function to rotate the motor, parameters: reference velocity and direction of motor
void Motor::rotate(double ref, int direction){
  if (INVERSE == false){
    digitalWrite(OUT1, LOW);
    digitalWrite(OUT2, HIGH);
  }
  else{
    digitalWrite(OUT1, HIGH);
    digitalWrite(OUT2, LOW);
    }
  Motor::pid_controller(ref);
  /*if (current_pwm < 0){
    digitalWrite(OUT1, -INVERSE);
    digitalWrite(OUT2, INVERSE);
    current_pwm = -current_pwm;
  }*/
  analogWrite(ENB, current_pwm);
}

//control algorithm to maintain the speed
void Motor::pid_controller(double ref){
  double err_now, vel_now, P, I, D;
  vel_now = Motor::get_angular_vel();
  err_now = ref - vel_now;
  /*if (abs(err_now) < deadzone){
    err_now = 0;
  }*/
  if (abs(err_now) < thres){
    err_integ = err_integ + err_now;
  }
  else{
    err_integ = 0;
  }
  P = err_now * Kp;
  I = err_integ * Ki * time_diff / MICRO_PER_SEC;
  D = (err_now - err_prev) * Kd * MICRO_PER_SEC / time_diff;
  current_pwm = (P + I + D) * scale;
  if (current_pwm > 255){
    current_pwm = 255;
  }
  if (current_pwm < 0){
    current_pwm = 0;
  }
  pwm_show = current_pwm;
  err_prev = err_now;
}

//function to read the current angular velocity of motors
double Motor::get_angular_vel(){
  time_now = micros();
  Motor::calc_angular_vel();
  tic_count_prev = tic_count;
  time_prev = time_now;
  return angular_vel;
}

//function to increment the tics for each motor.
void Motor::tic_counter(){
  tic_count++;
}



