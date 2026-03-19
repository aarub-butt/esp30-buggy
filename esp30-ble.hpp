#pragma once

#include <mbed.h>
#include "BuggyConfig.hpp"
#include <cstring>

class ble{
    private:
    
        static constexpr int command_buffer_size = sizeof(char[32]);

        // command
        int command_buffer_index;
        char command_buffer[command_buffer_size];

        bool readCommand(FSM::BLE_COMMAND *command_ptr);
        void clear_command_buffer();

        void ParseCommand(FSM::BLE_COMMAND *command_ptr);

    public:
        BufferedSerial pc;

        ble(PinName tx, PinName rx, int baudrate);
        void sendTelemetry(char* telemetry);
        void sendTelemetry(char* telemetry, FSM *fsm);
        bool getCommand(FSM::BLE_COMMAND *command_ptr);
}; 

