/**
Set the pin mode for pwm output pins
*/
void motor_setup();

/**
Move the robot forward
@int pwm: PWM value between 0 and 255- controls the speed of motors
*/
void forward(int pwm);

/**
Move the robot backwards
@int pwm: PWM value between 0 and 255- controls the speed of motors
*/
void backward(int pwm);

/**
Move the robot left
@int pwm: PWM value between 0 and 255- controls the speed of motors
*/
void left(int pwm);

/**
Move the robot right
@int pwm: PWM value between 0 and 255- controls the speed of motors
*/
void right(int pwm);

/**
Stop the robot
*/
void stop();

/**
Calculates PID control
@int speed: current speed
@int error: current error (tof sensor reading - setpoint)
@int prev_error: error from previous PID iteration
@int accum: accumulated error over all previous iterations
*/
int pid(int speed, int error, int prev_error, int accum);