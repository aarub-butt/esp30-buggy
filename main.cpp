/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include <mbed.h>
#include "SensorBoard.hpp"
#include "MotorDriveBoard.hpp"
#include "BuggyConfig.hpp"
#include "esp30-ble.hpp"
#include "reading-sensors.hpp"

const float  MotorDriveBoard::wheel_track_length = 0.22;
const float MotorDriveBoard::Motor::wheel_circumference = 3.14 * 0.0779;
const int MotorDriveBoard::Motor::pulses_per_revolution = 512;

float SensorBoard::LineSensor::alpha = 0.7;
float MotorDriveBoard::alpha = 0.7;

MotorDriveBoard::PID_controller MotorDriveBoard::steering_pid(1,0,0,1);
float MotorDriveBoard::dynamic_speed_constant = 1;
float MotorDriveBoard::max_speed = 1;

MotorConfig left_motor_config = 
{PC_4,PA_9 
,PB_15,PB_1 
,1,0,0,0.5};

MotorConfig right_motor_config = 
{PB_5,PA_8 
,PB_14,PB_13
,1,0,0,0.5};

SensorConfig sensor_config = 
{{
{A0,0.1f}
,{A1,0.2f}
,{A2,0.3f}
,{A3,0.4f}
,{A4,0.5f}
,{A5,0.6f}
}};



int main()
{

    ble pc(PA_11,PA_12,9600);
    MotorDriveBoard mdb(left_motor_config,right_motor_config, PB_4,20000);
    FSM fsm;
    SensorBoard sb(sensor_config);


    while (true){

        if (fsm.isNextCycle()){
            fsm.start_timestamp();
            fsm.ble_command.clear();
            pc.getCommand(&fsm, &fsm.ble_command, &mdb);
            
            switch (fsm.getProgramState()){
                
                case (STATE_STOP): {
                    mdb.setEnable(false);
                    mdb.setPWM(0.5,0.5);
                }
                break;

                case (STATE_NONE) : {
                    break;
                }

                case (STATE_ENCODER):
                    ReadingEncoder(&pc,&mdb,&fsm);
                    break;
                
                case (STATE_SENSOR):
                    ReadingSensors(&sb,&pc,&fsm);
                    break;

            
            }
        }
   
    }

}
