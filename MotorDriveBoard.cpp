#include "MotorDriveBoard.hpp"
#include "BuggyConfig.hpp"

// PID Controller methods
void MotorDriveBoard::PID_controller::reset(){
    integral = 0;
    previous_error = 0;
}
float MotorDriveBoard::PID_controller::calculate(float error, float dt){
    
    float P = kp * error;

    integral += error *dt;
    if (ki != 0){
        float max_I = output_limit / ki;
        if (integral > max_I/2) integral = max_I/2;
        if (integral < -max_I/2) integral = -max_I/2;
    }
    float I = ki*integral;

    float derivative = (error-previous_error)/dt;
    previous_error = error;
    float D = kd * derivative;

    float output = P + I + D;

    if (output > output_limit) output = output_limit;
    if (output < -output_limit) output = -output_limit;
    
    return output;
}
void MotorDriveBoard::PID_controller::setPid(float p, float i , float d){
    kp = p;
    ki = i;
    kd = d;
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
void MotorDriveBoard::Motor::calcSpeed(float time_elapsed){
    calcDistanceTravelled();
    speed = ((distance_travelled /  time_elapsed )*alpha) + (previous_speed *( 1-alpha));
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

void MotorDriveBoard::updateSpeeds(float dt){
    left_motor.current_pulse_count = left_motor.encoder.getPulses();
    right_motor.current_pulse_count = right_motor.encoder.getPulses();   
    
    left_motor.calcSpeed(dt);
    right_motor.calcSpeed(dt);

    left_motor.previous_pulse_count = left_motor.current_pulse_count;
    right_motor.previous_pulse_count = right_motor.current_pulse_count;
}

void MotorDriveBoard::getSpeeds(float*speeds){
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

void MotorDriveBoard::SetPwmFromTargetSpeed(float dt, float lt, float rt){
    left_motor.speed_error = lt - left_motor.speed;
    right_motor.speed_error = rt- right_motor.speed;
    
    float left_pwm = 0.5f + left_motor.speed_pid.calculate(left_motor.speed_error, dt);
    float right_pwm = 0.5f + right_motor.speed_pid.calculate(right_motor.speed_error, dt);

    setPWM(left_pwm, right_pwm);
}

void MotorDriveBoard::updateLineFollower(float error, float dt){
    float steering_output = steering_pid.calculate(error, dt);
    
    float base_speed = max_speed - (abs(error) * dynamic_speed_constant);

    float target_left_speed = base_speed + steering_output;
    float target_right_speed = base_speed - steering_output;

    SetPwmFromTargetSpeed(dt, target_left_speed, target_right_speed);
}

bool MotorDriveBoard::stop(float dt){
    if (abs(left_motor.speed) > 0.05f || abs(right_motor.speed) > 0.05f){
        SetPwmFromTargetSpeed(dt, 0.0f, 0.0f);
        return false;
    }
    return true;
}

void MotorDriveBoard::startRotate(float degree){
    _target_pulse_count = ((wheel_track_length/2)*((3.1415f/180)*degree))*(Motor::pulses_per_revolution/Motor::wheel_circumference);
    _rotation_stage = 0;
    _is_rotating = true;
}
bool MotorDriveBoard::updateRotate(float dt){
    if (_is_rotating != true){
        return true;
    }

    switch(_rotation_stage){
        case (0):
            if (stop(dt)){
                resetEncoders();
                _rotation_stage = 1;
            }
            break;
        
        case (1):
            if (rotate(dt)){
                _rotation_stage = 2;
            }
            break;

        case (2): 
            if (stop(dt)){
                _is_rotating = false;
                return true;
            }
            break;
    }
    return false;
}
bool MotorDriveBoard::rotate(float dt){
    
    int error = _target_pulse_count - ((left_motor.current_pulse_count - right_motor.current_pulse_count) /2);
    if (abs(error) < 10){
        return true;
    }

    float output = rotation_pid.calculate(error, dt);
    SetPwmFromTargetSpeed(dt, output, -output);

    return false;
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
isBipolar(motor_config.isBipolar), PWM(motor_config.pwm), encoder(motor_config.channel_A,motor_config.channel_B, NC, pulses_per_revolution,QEI::X4_ENCODING)
,speed_pid(motor_config.kp,motor_config.ki,motor_config.kd,motor_config.lim)
{
    speed = 0.0f;
    PWM_duty = 0.0f;
    previous_speed = 0.0f;

    resetEncoder();
}

MotorDriveBoard::PID_controller::PID_controller(float p, float i, float d, float lim) 
: kp(p), ki(i), kd(d), output_limit(lim){
    reset();
}

