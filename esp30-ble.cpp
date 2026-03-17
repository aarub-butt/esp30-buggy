#include "esp30-ble.hpp"
#include <cstring>

// Methods

void ble::clear_command_buffer(){
    memset((char*) command_buffer,0,command_buffer_size);
    command_buffer_index = 0;
}

bool ble::readCommand(char* command_ptr){
         
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
                strcpy(command_ptr, command_buffer);
                found_command = true;
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

bool ble::getCommand(char* command_ptr){
    
    if (pc.readable()){
        return readCommand(command_ptr);
    }
    return false;
}

void ble::sendTelemetry(char* telemetry){
    pc.write(telemetry,strlen(telemetry));
    pc.write("\r\n",2);
}

// Constructor 

ble::ble(PinName tx, PinName rx, int baudrate) : 
pc(tx,rx,baudrate){
    clear_command_buffer();
}


