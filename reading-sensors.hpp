#pragma once
#include <MotorDriveBoard.hpp>
#include <SensorBoard.hpp>
#include <esp30-ble.hpp>


void ReadingEncoder(ble* pc, MotorDriveBoard* mdb);
void ReadingSensors(SensorBoard* sensor_board, ble * pc);
