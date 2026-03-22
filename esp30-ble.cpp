#include "esp30-ble.hpp"
#include <cstdio>
#include <cstring>

// Methods

void ble::clear_command_buffer(){
    memset((char*) command_buffer,0,command_buffer_size);
    command_buffer_index = 0;
}

void ble::ParseCommand(FSM* fsm, FSM::BLE_COMMAND *command_ptr, MotorDriveBoard* mdb){
    
    if (strcmp(command_ptr->command,"none") == 0){
        fsm->nextState(STATE_NONE);
    }
    
    else if(sscanf(command_ptr->command, "rotate=%f", &command_ptr->value) == 1){
        fsm->nextState(STATE_ROTATE);
    }
    
    else if(strcmp(command_ptr->command,"encoder") == 0){
        fsm->nextState(STATE_ENCODER);
    }

    else if(strcmp(command_ptr->command,"sensor") == 0){
        fsm->nextState(STATE_SENSOR);
    }

    else if(strcmp(command_ptr->command,"enable") == 0){
        mdb->setEnable(!mdb->getEnable());
    }

    else if (sscanf(command_ptr->command, "lpwm=%f", &command_ptr->value) == 1) {
        float PWMs[2];
        mdb->getPWM(PWMs);
        mdb->setPWM(command_ptr->value, PWMs[1]);
    }

    else if (sscanf(command_ptr->command, "rpwm=%f", &command_ptr->value) == 1) {
        float PWMs[2];
        mdb->getPWM(PWMs);
        mdb->setPWM(PWMs[0],command_ptr->value);
    }

    else if(strcmp(command_ptr->command,"display") == 0){
        fsm->nextState(STATE_DISPLAY);
    }

}

bool ble::getCommand(FSM* fsm, FSM::BLE_COMMAND *command_ptr, MotorDriveBoard* mdb){
    
    if (pc.readable()){
        char c;
        bool found_command = false;

        while (pc.readable() == true){

            pc.read(&c, 1);

            if (command_buffer_index >= command_buffer_size){
                clear_command_buffer();
            }
            
            if (c == '\r' || c == '\n'){
                
                if (command_buffer_index > 0){
                    command_buffer[command_buffer_index]= '\0';
                    strcpy(command_ptr->command, command_buffer);
                    found_command = true;
                    ParseCommand(fsm, command_ptr, mdb);
                    clear_command_buffer();   
                    break;
                }

            }else{
                command_buffer[command_buffer_index] = c;
                command_buffer_index++;
            }

        }

        return found_command;
    }

    return false;
}

void ble::sendTelemetry(char* telemetry){
    pc.write(telemetry,strlen(telemetry));
}

void ble::sendTelemetry(char* telemetry, long long current_time, diff_time* cycle_timestamp){
    pc.write(telemetry,strlen(telemetry));
    
    int time_elapsed = getTimeElapsed_us(current_time, cycle_timestamp);
    snprintf(telemetry,32,"\r\n%d\r\n", time_elapsed);
    pc.write(telemetry, strlen(telemetry));
}

// Constructor 

ble::ble(PinName tx, PinName rx, int baudrate) : 
pc(tx,rx,baudrate){
    clear_command_buffer();
}


