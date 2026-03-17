/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include <mbed.h>
//#include "SensorBoard.hpp"
//#include "MotorDriveBoard.hpp"
#include "esp30-ble.hpp"

// wow a change

int main()
{

    char ble_command[32];
    Timer global_timer;
    global_timer.start();
    
    ble pc(PC_6,D9,9600);

    while (true){
        time_us.reset();

        if (pc.getCommand(ble_command)){
            pc.sendTelemetry(ble_command);
            char timer_buffer[20];
            snprintf(timer_buffer,20,"%lld", global_timer.elapsed_time().count());
            pc.sendTelemetry(timer_buffer);
       }
   
        ThisThread::sleep_for(200ms);
    }

}
