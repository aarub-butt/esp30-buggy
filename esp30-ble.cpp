#include "esp30-ble.hpp"
#include <cstdio>
#include <cstring>

// Methods

void ble::clear_command_buffer(){
    memset((char*) command_buffer,0,command_buffer_size);
    command_buffer_index = 0;
}

bool ble::readCommand(FSM::BLE_COMMAND *command_ptr){
         
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
                ParseCommand(command_ptr);
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

void ble::ParseCommand(FSM::BLE_COMMAND *command_ptr){
    
    if (strcmp(command_ptr->command,"menu") == 0){
        command_ptr->cmd = FSM::BLE_COMMAND::CMD_MENU;
    }
    
    else if(strcmp(command_ptr->command,"test") == 0){
        command_ptr->cmd = FSM::BLE_COMMAND::CMD_TEST;
    }
    
    else if(strcmp(command_ptr->command,"encoder") == 0){
        command_ptr->cmd = FSM::BLE_COMMAND::CMD_ENCODER;
    }

    else if(strcmp(command_ptr->command,"sensor") == 0){
        command_ptr->cmd = FSM::BLE_COMMAND::CMD_SENSOR;
    }

    else if(strcmp(command_ptr->command,"enable") == 0){
        command_ptr->cmd = FSM::BLE_COMMAND::CMD_ENABLE;
    }

    else if (sscanf(command_ptr->command, "lpwm=%f", &command_ptr->value) == 1) {
        command_ptr->cmd = FSM::BLE_COMMAND::CMD_SET_LEFT_PWM;
    }

    else if (sscanf(command_ptr->command, "rpwm=%f", &command_ptr->value) == 1) {
        command_ptr->cmd = FSM::BLE_COMMAND::CMD_SET_RIGHT_PWM;
    }

    else{
        command_ptr->clear();
    }

}

bool ble::getCommand(FSM::BLE_COMMAND *command_ptr){
    
    if (pc.readable()){
        return readCommand(command_ptr);
    }
    return false;
}

void ble::sendTelemetry(char* telemetry){
    pc.write(telemetry,strlen(telemetry));
}

void ble::sendTelemetry(char* telemetry, FSM* fsm){
    pc.write(telemetry,strlen(telemetry));
    
    int time_elapsed = getTimeElapsed_us(fsm, &fsm->cycle_timestamp);
    snprintf(telemetry,32,"\r\n%d\r\n", time_elapsed);
    pc.write(telemetry, strlen(telemetry));
}

// Constructor 

ble::ble(PinName tx, PinName rx, int baudrate) : 
pc(tx,rx,baudrate){
    clear_command_buffer();
}


