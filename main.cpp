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


int main()
{

    ble pc(PA_11,PA_12,9600);
    MotorDriveBoard mdb(BuggyConfig::left_motor_pins,BuggyConfig::right_motor_pins, PB_4,20000);
    FSM fsm;
    SensorBoard sb(BuggyConfig::sensor_pins, BuggyConfig::sensor_weights);


    while (true){

        if (fsm.isNextCycle()){
            fsm.start_timestamp();

            switch (fsm.getProgramState()){
                
                case (BuggyConfig::STATE_MENU): {
                    if (pc.getCommand(&fsm.ble_command)){
                        switch (fsm.ble_command.cmd){
                            
                            case (FSM::BLE_COMMAND::CMD_ENCODER):
                                fsm.nextState(BuggyConfig::STATE_ENCODER);
                                break;
                            
                            case (FSM::BLE_COMMAND::CMD_TEST):
                                fsm.nextState(BuggyConfig::STATE_TEST);
                                break;
                            
                            case (FSM::BLE_COMMAND::CMD_SENSOR):
                                fsm.nextState(BuggyConfig::STATE_SENSOR);
                                break;

                            default:
                                break;
                        }
                    }
                }
                break;
                
                case (BuggyConfig::STATE_ENCODER):
                    ReadingEncoder(&pc,&mdb,&fsm);
                    break;

                case (BuggyConfig::STATE_TEST):
                    TestParts(&pc,&mdb,&fsm);
                    break;
                
                case (BuggyConfig::STATE_SENSOR):
                    ReadingSensors(&sb,&pc,&fsm);
                    break;

                default:
                    break;
            }


        }


/*
        if (pc.getCommand(ble_command)){
            pc.sendTelemetry(ble_command);
            char timer_buffer[32];
            snprintf(timer_buffer,20,"%lld", global_timer.elapsed_time().count());
            pc.sendTelemetry(timer_buffer);
       }

       */
   
    }

}
