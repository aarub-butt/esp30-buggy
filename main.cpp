/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include <cstdlib>
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
MotorDriveBoard::PID_controller MotorDriveBoard::rotation_pid(0.002f,0,0,0.3);
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

    ble pc(PA_11,PA_12,115200);
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
                    float dt;
                    getTimeElapsed(fsm.global_timer.elapsed_time().count(),&mdb.times, &dt);
                    if (mdb.stop(dt)) fsm.nextState(STATE_NONE);
                }
                break;
                case (STATE_NONE) : 
                    break;
                case (STATE_DISPLAY):{
                    char telemetry[telemetry_size*3];
                    snprintf(telemetry, telemetry_size*3, 
                    "e=%d,ks=%.2f\r\n\
                    lpwm=%.2f,kp=%.2f,ki=%.2f,kd=%.2f\r\n\
                    rpwm=%.2f,kp=%.2f,ki=%.2f,kd=%.2f\r\n"
                    ,mdb.getEnable(), mdb.dynamic_speed_constant,
                    mdb.left_motor.PWM_duty, mdb.left_motor.speed_pid.kp,mdb.left_motor.speed_pid.ki,mdb.left_motor.speed_pid.kd,
                    mdb.right_motor.PWM_duty, mdb.right_motor.speed_pid.kp,mdb.right_motor.speed_pid.ki,mdb.right_motor.speed_pid.kd);
                  
                    pc.sendTelemetry(telemetry);

                    ThisThread::sleep_for(100ms);
                    memset((char*) telemetry,0,telemetry_size*3);
                    
                    snprintf(telemetry, telemetry_size*3,
                    "b:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n\
                     w:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
                    sb.sensors[0].black,sb.sensors[1].black,sb.sensors[2].black,sb.sensors[3].black,sb.sensors[4].black,sb.sensors[5].black,
                    sb.sensors[0].white,sb.sensors[1].white,sb.sensors[2].white,sb.sensors[3].white,sb.sensors[4].white,sb.sensors[5].white);
                                     
                    ThisThread::sleep_for(100ms);
                    
                    fsm.nextState(STATE_NONE);
                }
                break;
                case (STATE_ENCODER):
                    ReadingEncoder(&pc,&mdb,&fsm);
                    break;
                
                case (STATE_SENSOR):
                    ReadingSensors(&sb,&pc,&fsm);
                    break;

                case (STATE_LINE_FOLLOWING): {
                    float line_error; 
                    long long current_time = fsm.global_timer.elapsed_time().count();
                    float dt;
                    getTimeElapsed(current_time, &mdb.times, &dt);

                    if (sb.getLinePosition(&line_error, current_time)){
                        mdb.updateLineFollower(line_error,dt);
                        if (fsm.shouldPrint()){
                            char telemetry[telemetry_size];
                            
                            snprintf(telemetry,telemetry_size,
                            "%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
                            line_error,
                            mdb.left_motor.speed ,mdb.left_motor.speed_error,
                            mdb.right_motor.speed ,mdb.right_motor.speed_error);

                            pc.sendTelemetry(telemetry,fsm.global_timer.elapsed_time().count(), &fsm.cycle_timestamp);
                        }
                    }else{
                        if (current_time - SensorBoard::line_break.start_time > 100000){
                            fsm.nextState(STATE_STOP);
                        }
                    }
                }
                break;
                case (STATE_ROTATE) :{
                    float dt;
                    getTimeElapsed(fsm.global_timer.elapsed_time().count(), &mdb.times, &dt);
                    if (fsm.isNotRepeatState()){
                        mdb.startRotate(fsm.ble_command.value);
                    }
                    if(mdb.updateRotate(dt)){
                        fsm.nextState(fsm.return_state);
                    }
                }
                break;
                case (STATE_CALIBRATE):{
                    float dt;
                    getTimeElapsed(fsm.global_timer.elapsed_time().count(), &mdb.times, &dt);

                    sb.calibrate();

                    if (fsm.isNotRepeatState()){
                        mdb.startRotate(720);
                    }
                    if(mdb.updateRotate(dt)){
                        fsm.nextState(STATE_DISPLAY);
                    }
                }
                break;
                case (STATE_INVALID):{
                    if (fsm.isNotRepeatState()){
                        char telemetry[] = "INVALID STATE";
                        pc.sendTelemetry(telemetry);
                    }
                    fsm.nextState(fsm.getProgramState());
                }
                break;  
                default:
                    fsm.nextState(STATE_INVALID);
                    break;
                    
            }
        }
   
    }

}
