/**
*/
void motor_setup();

/**
Move the robot forward
@int pwm: PWM value between 0 and 255- controls the speed
*/
void forward(int pwm);

/**
Move the robot backwards
@int pwm: PWM value between 0 and 255- controls the speed
*/
void backward(int pwm);

/**
Stop the robot
@int pwm: 
@int ms: duration (ms)
*/
void stop();

/**
Performs PID given the current speed and error
@int speed
@int error
@int prev_error
*/
int pid(int speed, int error, int prev_error, int accum);