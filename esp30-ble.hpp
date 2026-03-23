#pragma once

#include <mbed.h>
#include "BuggyConfig.hpp"
#include "MotorDriveBoard.hpp"
#include <cstring>

class ble{
    private:
    
        static constexpr int command_buffer_size = sizeof(char[telemetry_size]);

        // command
        int command_buffer_index;
        char command_buffer[command_buffer_size];

        void clear_command_buffer();

        void ParseCommand(FSM* fsm, FSM::BLE_COMMAND *command_ptr, MotorDriveBoard* mdb);

    public:
        BufferedSerial pc;

        ble(PinName tx, PinName rx, int baudrate);
        void sendTelemetry(char* telemetry);
        void sendTelemetry(char* telemetry, long long current_time, diff_time* cycle_timestamp);
        bool getCommand(FSM* fsm, FSM::BLE_COMMAND *command_ptr, MotorDriveBoard* mdb);
}; 

