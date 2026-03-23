#include "esp30-ble.hpp"
#include <cstdio>
#include <cstring>

// Methods

void ble::clear_command_buffer(){
    memset((char*) command_buffer,0,command_buffer_size);
    command_buffer_index = 0;
}

void ble::ParseCommand(FSM* fsm, FSM::BLE_COMMAND *command_ptr, MotorDriveBoard* mdb){
    
    float a,b,c;

    if (strcmp(command_ptr->command,"none") == 0){
        fsm->nextState(STATE_NONE);
    }

    if (strcmp(command_ptr->command,"stop") == 0){
        fsm->nextState(STATE_STOP);
    }
    
    else if(sscanf(command_ptr->command, "rotate=%f", &command_ptr->value) == 1){
        fsm->return_state = fsm->getPreviousProgramState();
        fsm->nextState(STATE_ROTATE);
    }
    
    else if(strcmp(command_ptr->command,"encoder") == 0){
        fsm->nextState(STATE_ENCODER);
    }

    else if(strcmp(command_ptr->command,"sensor") == 0){
        fsm->nextState(STATE_SENSOR);
    }

    else if(strcmp(command_ptr->command,"e") == 0){
        mdb->setEnable(!mdb->getEnable());
    }

    else if (sscanf(command_ptr->command, "pwm=%f,%f", &a, &b) == 1) {
        mdb->setPWM(a, b);
    }

    else if (sscanf(command_ptr->command, "lm=%f,%f,%f", &a,&b,&c) == 1) {
        mdb->left_motor.speed_pid.setPid(a, b, c);
    }

    else if (sscanf(command_ptr->command, "rm=%f,%f,%f", &a,&b,&c) == 1) {
        mdb->right_motor.speed_pid.setPid(a, b, c);
    }

    else if (sscanf(command_ptr->command, "steer=%f,%f,%f", &a,&b,&c) == 1) {
        mdb->steering_pid.setPid(a, b, c);
    }

    else if(strcmp(command_ptr->command,"tel") == 0){
        fsm->toggleTelemetry();
    }

    else if(strcmp(command_ptr->command,"display") == 0){
        fsm->nextState(STATE_DISPLAY);
    }

    else if(strcmp(command_ptr->command,"line") == 0){
        fsm->nextState(STATE_LINE_FOLLOWING);
    }

    else if(strcmp(command_ptr->command,"calibrate") == 0){
        fsm->nextState(STATE_CALIBRATE);
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
    
    int dt;
    getTimeElapsed(current_time, cycle_timestamp, &dt);

    snprintf(telemetry,telemetry_size,"%d\r\n", dt);
    pc.write(telemetry, strlen(telemetry));
}


// Constructor 

ble::ble(PinName tx, PinName rx, int baudrate) : 
pc(tx,rx,baudrate){
    clear_command_buffer();
}


