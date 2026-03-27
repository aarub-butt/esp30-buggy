#pragma once
#include <mbed.h>
#include "MotorDriveBoard.hpp"
#include "SensorBoard.hpp"
#include "esp30-ble.hpp"
#include "BuggyConfig.hpp"
#include <cstring>

/**
* @file reading-sensors.hpp
* @brief Encapsulates the testing of reading sensor outputs
*/

/**
* @brief Read data from encoder and send the telemetry via BLE to device
* @details Reads for both left and right of buggy their speed and current pulse counts and
*sends via bluetooth to device with an optional included timestamp to see if these processes are too taxing
* @param[in,out] pc Pointer to @ref ble object for sending telemetry
* @param[in] mdb Pointer to @ref MotorDriveBoard object to read encoder/speed data
* @param[in] fsm Pointer to @ref FSM object to calculate timestamp and whether to send telemetry
*/
void ReadingEncoder(ble* pc, MotorDriveBoard* mdb, FSM* fsm);



/**
* @brief Read data from encoder and send the telemetry via BLE to device
* @details Reads for both left and right of buggy their speed and current pulse counts and
*sends via bluetooth to device with an optional included timestamp to see if these processes are too taxing
* @param[in,out] pc Pointer to @ref ble object for sending telemetry
* @param[in] sensor_board Pointer to @ref SensorBoard object to read sensor data
* @param[in] fsm Pointer to @ref FSM object to calculate timestamp and whether to send telemetry
*/
void ReadingSensorBoard(SensorBoard* sensor_board, ble * pc, FSM* fsm);
