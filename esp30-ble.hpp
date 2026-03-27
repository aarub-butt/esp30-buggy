#pragma once

#include <mbed.h>
#include "BuggyConfig.hpp"
#include "MotorDriveBoard.hpp"
#include <cstring>

/**
 * @file esp30-ble.hpp
 * @brief BLE handler for remote control and telemetry
*/

/**
 * @class ble
 * @brief Manages communication with the buggy's BLE module.
 * @details Responsible for reading incoming commands, parsing commands 
 * to update state and parameters, and sending buggy telemetry.
 */
class ble{
    private:
    
        static constexpr int command_buffer_size = sizeof(char[telemetry_size]); ///< Maximum size for a command

        int command_buffer_index; ///< Current position in @ref command_buffer
        char command_buffer[command_buffer_size]; ///< Storage for incoming characters 

        /**
        * @brief Clears @ref command_buffer and @ref command_buffer_index
        */
        void clear_command_buffer();

        /**
        * @brief Parse commands to find valid commands and apply the commands.
        */
        void ParseCommand(FSM* fsm, FSM::BLE_COMMAND *command_ptr, MotorDriveBoard* mdb);

    public:
        BufferedSerial pc; ///< UART object for connecting to BLE module

        /**
        * @brief Constructor for ble object
        * @param[in] tx Pin to send data to BLE module
        * @param[in] rx Pin to receive data from BLE module
        * @param[in] baudrate Baudrate of BLE module 
        */
        ble(PinName tx, PinName rx, int baudrate); 
        

        /** 
        * @brief Sends telemetry device
        */
        void sendTelemetry(char* telemetry);
        
        /**
         * @overload
         * @brief Sends telemetry with an appended timestamp
         */
        void sendTelemetry(char* telemetry, long long current_time, diff_time* cycle_timestamp);
        

        /**
         * @brief Check for incoming BLE commands.
         * @param[out] fsm Pointer to update system state
         * @param[out] command_ptr Pointer to store parsed command.
         * @param[out] mdb Pointer to for real-time tuning of motor values.
         * @return True if a full command was successfully received and parsed.
         */
        bool getCommand(FSM* fsm, FSM::BLE_COMMAND *command_ptr, MotorDriveBoard* mdb);
}; 

