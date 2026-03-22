#include "MotorDriveBoard.hpp"
#include "BuggyConfig.hpp"

// PID Controller methods
void MotorDriveBoard::PID_controller::reset(){
    integral = 0;
    previous_error = 0;
}
float MotorDriveBoard::PID_controller::calculate(float error, int dt){
    
    float P = kp * error;

    integral += error *dt;
    float I = ki*integral;

    float derivative = (error-previous_error)/dt;
    previous_error = error;
    float D = kd * derivative;

    float output = P + I + D;

    if (output > output_limit) output = output_limit;
    if (output < -output_limit) output = -output_limit;
    
    return output;
}

// Motor Methods
void MotorDriveBoard::Motor::resetEncoder(){
    distance_travelled = 0;
    previous_pulse_count =0;
    current_pulse_count = 0;
    encoder.reset();
}
void MotorDriveBoard::Motor::calcDistanceTravelled(){
    distance_travelled = wheel_circumference * (((float) current_pulse_count - previous_pulse_count)/pulses_per_revolution);
}
void MotorDriveBoard::Motor::calcSpeed(int time_elapsed){
    calcDistanceTravelled();
    speed = (((distance_travelled) / ( ( time_elapsed )/1000000.0f))*alpha) + (previous_speed *( 1-alpha));
    previous_speed = speed;
}

// MotorDriveBoard Methods

void MotorDriveBoard::setEnable(bool isEnable){
    enable = isEnable;
}
bool MotorDriveBoard::getEnable(){
    return enable;
}


void MotorDriveBoard::setPWM(float left, float right){
    // mirror variables
    left_motor.PWM_duty = left;
    right_motor.PWM_duty = right;

    // actual change pwm
    left_motor.PWM.write(left);
    right_motor.PWM.write(right);
}
void MotorDriveBoard::getPWM(float* PWMs){
    PWMs[0] = left_motor.PWM_duty;
    PWMs[1] = right_motor.PWM_duty;
}

void MotorDriveBoard::updateSpeeds(int dt){
    left_motor.current_pulse_count = left_motor.encoder.getPulses();
    right_motor.current_pulse_count = right_motor.encoder.getPulses();   
    
    left_motor.calcSpeed(dt);
    right_motor.calcSpeed(dt);

    left_motor.previous_pulse_count = left_motor.current_pulse_count;
    right_motor.previous_pulse_count = right_motor.current_pulse_count;
}
void MotorDriveBoard::getSpeeds(int dt, float*speeds){
    updateSpeeds(dt);
    speeds[0] = left_motor.speed;
    speeds[1] = right_motor.speed;
}

void MotorDriveBoard::resetEncoders(){
    left_motor.resetEncoder();
    right_motor.resetEncoder();
}
void MotorDriveBoard::getPulseCounts(int* pulse_counts){
    pulse_counts[0] = left_motor.current_pulse_count;
    pulse_counts[1] = right_motor.current_pulse_count;
}

void MotorDriveBoard::updateLineFollower(float error, long long current_time){
    int dt = (getTimeElapsed_us(current_time, &times))/1000000;

    float steering_output = steering_pid.calculate(error, dt);
    
    float base_speed = max_speed - (abs(error) * dynamic_speed_constant);

    float target_left_speed = base_speed + steering_output;
    float target_right_speed = base_speed - steering_output;

    updateSpeeds(dt);
    float left_pwm = 0.5f + left_motor.speed_pid.calculate(target_left_speed - left_motor.speed, dt);
    float right_pwm = 0.5f + right_motor.speed_pid.calculate(target_right_speed - right_motor.speed, dt);

    setPWM(left_pwm, right_pwm);
}

// Constructors

MotorDriveBoard::MotorDriveBoard(MotorConfig left_motor_config, MotorConfig right_motor_config, PinName e, float pwmFrequency) 
: left_motor(left_motor_config), right_motor(right_motor_config), enable(e)
{
    setEnable(false);
    
    right_motor.isBipolar = true;
    left_motor.isBipolar = true;

    this->PWM_frequency = pwmFrequency;
    float PWM_period = 1/PWM_frequency;
    left_motor.PWM.period(PWM_period);
    right_motor.PWM.period(PWM_period);

    setPWM(0.5f,0.5f);
}

MotorDriveBoard::Motor::Motor(MotorConfig motor_config) :
isBipolar(motor_config.isBipolar), PWM(motor_config.pwm), encoder(motor_config.channel_A,motor_config.channel_B, NC, pulses_per_revolution)
,speed_pid(motor_config.kp,motor_config.ki,motor_config.kd,motor_config.lim)
{
    speed = 0.0f;
    PWM_duty = 0.0f;
    RPM = 0.0f;
    previous_speed = 0.0f;

    resetEncoder();
}

MotorDriveBoard::PID_controller::PID_controller(float p, float i, float d, float lim) 
: kp(p), ki(i), kd(d), output_limit(lim){
    reset();
}

