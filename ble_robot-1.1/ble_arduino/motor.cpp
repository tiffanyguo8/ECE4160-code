#include "motor.h"

/*
    6+7:
    12+13: 33 forward, 38 backwards deadband in air
*/

void motor_setup() {
  pinMode(6, OUTPUT); // ccw
  pinMode(7, OUTPUT); // clockwise
  pinMode(12, OUTPUT); // ccw
  pinMode(13, OUTPUT); // clockwise
}

void forward(int pwm) {
  analogWrite(7, 0); 
  analogWrite(12, 0);
  analogWrite(6, pwm);
  analogWrite(13, pwm-25); 
}

void backward(int pwm) {
  analogWrite(13, 0); 
  analogWrite(12, pwm);
  // delay(80);
  analogWrite(6, 0);
  analogWrite(7, pwm); 
}

void left(int pwm) {
  analogWrite(13, pwm); 
  analogWrite(12, 0);
  analogWrite(6, 0);
  analogWrite(7, pwm); 
}

void right(int pwm) {
  analogWrite(13, 0); 
  analogWrite(12, pwm);
  analogWrite(6, pwm);
  analogWrite(7, 0); 
}

void stop() {
  analogWrite(6, 0);
  analogWrite(7, 0); 
  analogWrite(12, 0);
  analogWrite(13, 0); 
}

int pid(int speed, int error, int prev_error, int accum)
{
    float kp = 0.08;
    float kd = 0.2;
    float ki = 0.125;
    int d_e = error - prev_error;
    if(accum > 100) accum = 100;
    else if(accum < -100) accum = -100;
    speed = kp*error + kd*d_e + ki*accum;
    prev_error = error;

    if(speed > 255) return 255;
    else if(speed < -255) return -255;
    else if(speed >= 0 && speed < 10) return 0;
    else if(speed > 0 && speed < 40) return 40;
    else if(speed < 0 && speed > -80) return -80;
    else return speed;
}