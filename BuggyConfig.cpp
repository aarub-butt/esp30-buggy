#include "BuggyConfig.hpp"

// FSM Methods / constructor

Timer FSM::global_timer;

void FSM::InitiateCycle(){
    var_isNextCycle = true;
}

FSM::FSM(){
    programState = STATE_NONE;
    previousProgramState = STATE_NONE;
    buggy_rate.attach(callback(this,&FSM::InitiateCycle),buggy_period);
    var_isNextCycle = false;

    global_timer.start();
    var_shouldPrint = 1;
    should_send_telemetry = false;
}

void FSM::toggleTelemetry(){
    should_send_telemetry = !should_send_telemetry;
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
        previousProgramState = programState;
        return true;
    }
}

void FSM::nextState(ProgramState nextState){
    previousProgramState = programState;
    programState = nextState;
}

void FSM::start_timestamp(){
    cycle_timestamp.current_time = global_timer.elapsed_time().count();
    cycle_timestamp.previous_time = cycle_timestamp.current_time;
}

ProgramState FSM::getProgramState(){
    return programState;
}

ProgramState FSM::getPreviousProgramState(){
    return previousProgramState;
}

bool FSM::shouldPrint(){
    if (var_shouldPrint >= 20 && should_send_telemetry == true){
        var_shouldPrint = 1;
        return true;
    }else{
        var_shouldPrint++;
        return false;
    }
}

// diff time 

diff_time::diff_time(){
    previous_time = 0;
    current_time = 0;
}

void getTimeElapsed(long long current_time, diff_time *times, float* dt_s){
    
    times->current_time = current_time;
    long long time_elapsed = (times->current_time - times->previous_time);
    times->previous_time = times->current_time;

    *dt_s = time_elapsed/1000000.0f; 
}
void getTimeElapsed(long long current_time, diff_time *times, int* dt_us){
    
    times->current_time = current_time;
    int time_elapsed = (times->current_time - times->previous_time);
    times->previous_time = times->current_time;

    *dt_us = time_elapsed; 
}

// BLE_COMMAND

void FSM::BLE_COMMAND::clear(){
    memset((char*) command,0,64);
    value = 0;
}

FSM::BLE_COMMAND::BLE_COMMAND(){
    clear();
}