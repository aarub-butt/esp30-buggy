#include "BuggyConfig.hpp"

BuggyConfig::MotorPins BuggyConfig::left_motor_pins = 
{PC_4,PA_9 ,PB_15,PB_1};
BuggyConfig::MotorPins BuggyConfig::right_motor_pins = 
{PB_5,PA_8 ,PB_14,PB_13};

BuggyConfig::SensorPins BuggyConfig::sensor_pins = 
{A0,A1,A2,A3,A4,A5};
BuggyConfig::SensorWeights BuggyConfig::sensor_weights =
{0.1f,0.2f,0.3f,0.4f,0.5f,0.6f}; 


// FSM Methods / constructor

Timer FSM::global_timer;

void FSM::InitiateCycle(){
    var_isNextCycle = true;
}

FSM::FSM(){
    programState = BuggyConfig::STATE_MENU;
    previousProgramState = BuggyConfig::STATE_NONE;
    buggy_rate.attach(callback(this,&FSM::InitiateCycle),buggy_period);
    var_isNextCycle = false;

    global_timer.start();
    var_shouldPrint = 1;
}

bool FSM::isNextCycle(){
    bool return_val = false;
    
    if (var_isNextCycle == true){
        return_val = true;
    }
    
    var_isNextCycle = false;
    return return_val;
}

bool FSM::isNotRepeatState(){
    if (programState == previousProgramState){
        return false;
    }else{
        return true;
    }
}

void FSM::nextState(BuggyConfig::ProgramState nextState){
    previousProgramState = programState;
    programState = nextState;
}

void FSM::start_timestamp(){
    cycle_timestamp.current_time = global_timer.elapsed_time().count();
    cycle_timestamp.previous_time = cycle_timestamp.current_time;
}

BuggyConfig::ProgramState FSM::getProgramState(){
    return programState;
}

bool FSM::shouldPrint(){
    if (var_shouldPrint == 100){
        var_shouldPrint = 1;
        return true;
    }else{
        var_shouldPrint++;
        return false;
    }
}

// diff time 

BuggyConfig::diff_time::diff_time(){
    previous_time = 0;
    current_time = 0;
}

int getTimeElapsed_us(FSM *fsm, BuggyConfig::diff_time *times){
    
    times->current_time = fsm->global_timer.elapsed_time().count();
    int time_elapsed = times->current_time - times->previous_time;
    times->previous_time = times->current_time;

    return time_elapsed;
}

// BLE_COMMAND

void FSM::BLE_COMMAND::clear(){
    memset((char*) command,0,64);
    value = 0;
    cmd = CMD_INVALID;
}

FSM::BLE_COMMAND::BLE_COMMAND(){
    clear();
}