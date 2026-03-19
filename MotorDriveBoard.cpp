#include "MotorDriveBoard.hpp"

// static variables

const float  MotorDriveBoard::wheel_track_length = 0.22;
const float MotorDriveBoard::Motor::wheel_circumference = 3.14 * 0.0779;
const int MotorDriveBoard::Motor::pulses_per_revolution = 512;
                        

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

void MotorDriveBoard::Motor::calcSpeed(int *time_elapsed){
    calcDistanceTravelled();
    speed = (distance_travelled) / ( ( (long) *time_elapsed )/1000000.0f);
}

// MotorDriveBoard Methods

void MotorDriveBoard::setEnable(bool isEnable){
    enable = isEnable;
}

bool MotorDriveBoard::getEnable(){
    return enable;
}

void MotorDriveBoard::setIsBipolar(bool isBipolar){
    right_motor.isBipolar = isBipolar;
    left_motor.isBipolar = isBipolar;
}

void MotorDriveBoard::set_PWM_frequency(float PWM_frequency){
    this->PWM_frequency = PWM_frequency;
    float PWM_period = 1/PWM_frequency;
    left_motor.PWM.period(PWM_period);
    right_motor.PWM.period(PWM_period);
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

void MotorDriveBoard::getSpeed(float* speeds){
    speeds[0] = left_motor.speed;
    speeds[1] = right_motor.speed;
}

void MotorDriveBoard::calcSpeed(FSM *fsm){

    left_motor.current_pulse_count = left_motor.encoder.getPulses();   
    right_motor.current_pulse_count = right_motor.encoder.getPulses();    
    
    int time_elapsed = getTimeElapsed_us(fsm, &times);
    left_motor.calcSpeed(&time_elapsed);
    right_motor.calcSpeed(&time_elapsed);

    left_motor.previous_pulse_count = left_motor.current_pulse_count;
    right_motor.previous_pulse_count = right_motor.current_pulse_count;
}

void MotorDriveBoard::resetEncoders(){
    left_motor.resetEncoder();
    right_motor.resetEncoder();
}

void MotorDriveBoard::getPulseCounts(int* pulse_counts){
    pulse_counts[0] = left_motor.current_pulse_count;
    pulse_counts[1] = right_motor.current_pulse_count;
}

// Constructors

MotorDriveBoard::MotorDriveBoard(BuggyConfig::MotorPins left_motor_pins, BuggyConfig::MotorPins right_motor_pins, PinName e, float PWM_frequency) 
: left_motor(left_motor_pins), right_motor(right_motor_pins), enable(e)
{
    setEnable(false);
    setIsBipolar(true);
    set_PWM_frequency(PWM_frequency);
    setPWM(0.5f,0.5f);
}

MotorDriveBoard::Motor::Motor(BuggyConfig::MotorPins motor_pins) :
isBipolar(motor_pins.isBipolar), PWM(motor_pins.pwm), encoder(motor_pins.channel_A,motor_pins.channel_B, NC, pulses_per_revolution)
{
    speed = 0.0f;
    PWM_duty = 0.0f;
    RPM = 0.0f;

    resetEncoder();
}

