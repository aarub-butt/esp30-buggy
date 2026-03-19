#pragma once
#include <mbed.h>
#include "MotorDriveBoard.hpp"
#include "SensorBoard.hpp"
#include "esp30-ble.hpp"
#include "BuggyConfig.hpp"
#include <cstring>


void ReadingEncoder(ble* pc, MotorDriveBoard* mdb, FSM* fsm);
void ReadingSensors(SensorBoard* sensor_board, ble * pc, FSM* fsm);
void TestParts(ble* pc, MotorDriveBoard* mdb, FSM* fsm);
