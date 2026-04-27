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
const int MotorDriveBoard::Motor::pulses_per_revolution = 256*4;

float SensorBoard::LineSensor::alpha = 0.7;
float MotorDriveBoard::alpha = 0.1;

MotorDriveBoard::PID_controller MotorDriveBoard::steering_pid(3,0,0,1);
float MotorDriveBoard::dynamic_speed_constant = 1;
float MotorDriveBoard::max_speed = 0.4f;

MotorConfig left_motor_config = 
{PC_4,PA_9 
,PB_15,PB_1 
,0.2,1,0,0.5};
//pa8 to d2
MotorConfig right_motor_config = 
{PB_5,D2
,PB_14,PB_13
,0.2,1,0,0.5};

SensorConfig sensor_config = 
{{
{A5,-1.0f}
,{A4,-0.6f}
,{A3,-0.2f}
,{A2,0.2f}
,{A1,0.6f}
,{A0,1.0f}
}};

int main()
{

    ble pc(PA_11,PA_12,9600);
    MotorDriveBoard mdb(left_motor_config,right_motor_config, PB_4,20000);
    FSM fsm;
    SensorBoard sb(sensor_config);

    int heartbeat_count = 0;
    char heartbeat[] = "heartbeat\r\n";

    while (true){

        if (fsm.isNextCycle()){
            
            // check if ble connection is working
            /*
            heartbeat_count++;
            if (heartbeat_count >= 1000){
                pc.sendTelemetry(heartbeat);
                heartbeat_count = 0;
            } */

            fsm.start_timestamp();
            fsm.ble_command.clear();
            pc.getCommand(&fsm, &fsm.ble_command, &mdb);

            float dt;
            long long current_time = fsm.global_timer.elapsed_time().count();
            getTimeElapsed(current_time, &mdb.times, &dt);
            mdb.updateSpeeds(dt);
            
            switch (fsm.getProgramState()){
                
                case (STATE_STOP): {
                    if (fsm.isNotRepeatState()){
                        char state[] = "stop\r\n";
                        pc.sendTelemetry(state);
                    }
                    if (mdb.stop(dt)) fsm.nextState(STATE_NONE);
                }
                break;
                case (STATE_NONE) : {
                    if (fsm.isNotRepeatState()){
                        char state[] = "none\r\n";
                        pc.sendTelemetry(state);
                    }
                }
                    break;
                case (STATE_DISPLAY):{
                    char telemetry[telemetry_size*3];
                    snprintf(telemetry, telemetry_size*3,
                    "e=%d,ks=%.2f\r\n"
                    "lpwm=%.2f,kp=%.2f,ki=%.2f,kd=%.2f\r\n"
                    "rpwm=%.2f,kp=%.2f,ki=%.2f,kd=%.2f\r\n"
                    ,mdb.getEnable(), mdb.dynamic_speed_constant,
                    mdb.left_motor.PWM_duty, mdb.left_motor.speed_pid.kp,mdb.left_motor.speed_pid.ki,mdb.left_motor.speed_pid.kd,
                    mdb.right_motor.PWM_duty, mdb.right_motor.speed_pid.kp,mdb.right_motor.speed_pid.ki,mdb.right_motor.speed_pid.kd);
                  
                    pc.sendTelemetry(telemetry);
                    ThisThread::sleep_for(500ms);
                    memset((char*) telemetry,0,telemetry_size*3);
                    
                    snprintf(telemetry, telemetry_size*3,
                    "b:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n"
                    "w:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
                    sb.sensors[0].black,sb.sensors[1].black,sb.sensors[2].black,sb.sensors[3].black,sb.sensors[4].black,sb.sensors[5].black,
                    sb.sensors[0].white,sb.sensors[1].white,sb.sensors[2].white,sb.sensors[3].white,sb.sensors[4].white,sb.sensors[5].white);
                    
                    pc.sendTelemetry(telemetry);
                    memset((char*) telemetry,0,telemetry_size*3);
                    ThisThread::sleep_for(500ms);

                    snprintf(telemetry, telemetry_size*3,
                    "st:kp=%.2f,ki=%.2f,kd=%.2f,ms=%.2f\r\n",
                    mdb.steering_pid.kp,mdb.steering_pid.ki,mdb.steering_pid.kd,mdb.max_speed);

                    pc.sendTelemetry(telemetry);
                    memset((char*) telemetry,0,telemetry_size*3);
                    ThisThread::sleep_for(500ms);

                    fsm.nextState(STATE_NONE);
                }
                break;
                case (STATE_ENCODER):
                    ReadingEncoder(&pc,&mdb,&fsm);
                    break;
                
                case (STATE_SENSOR):
                    ReadingSensorBoard(&sb,&pc,&fsm);
                    break;

                case (STATE_LINE_FOLLOWING): {
                    float line_error; 

                    static bool is_line_break;
                    static long long line_break_start_time;
                    if (fsm.isNotRepeatState()){
                        is_line_break = false;
                        line_break_start_time = 0;
                        mdb.steering_pid.reset();
                        mdb.right_motor.speed_pid.reset();
                        mdb.left_motor.speed_pid.reset();

                        char state[] = "line following";
                        pc.sendTelemetry(state);
                    }

                    if (sb.getLinePosition(&line_error)){
                        is_line_break = false;
                        mdb.updateLineFollower(line_error,dt);
                        if (fsm.shouldPrint()){
                            char telemetry[telemetry_size];
                            
                            snprintf(telemetry,telemetry_size,
                            ",%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
                            line_error,
                            mdb.left_motor.speed ,mdb.left_motor.speed_error,
                            mdb.right_motor.speed ,mdb.right_motor.speed_error);
                            pc.sendTelemetry(telemetry);
                            //pc.sendTelemetry(telemetry,fsm.global_timer.elapsed_time().count(), &fsm.cycle_timestamp);
                        }

                    }else{

                        if (is_line_break == false){
                            is_line_break = true;
                            line_break_start_time = current_time;
                        }

                        if ( (current_time - line_break_start_time) > (100000)){
                            fsm.nextState(STATE_STOP);
                        }
                    }
                }
                break;
                case (STATE_ROTATE) :{
                    static bool is_rotate;
                    if (fsm.isNotRepeatState()){
                        mdb.startRotate(fsm.ble_command.value);
                        is_rotate = false;
                    }

                    if (is_rotate == false){
                        if (mdb.stop(dt)){
                            is_rotate = true;
                            mdb.resetEncoders();
                        }

                    }else{
                        if(mdb.updateRotate(dt)){
                            fsm.nextState(fsm.return_state);
                        }
                    }
                }
                break;
                case (STATE_CALIBRATE):{
                    sb.calibrate();
                    if (fsm.isNotRepeatState()){
                        mdb.startRotate(360);
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
                }
                break;

                case (STATE_SPEED):{
                    static float target_speed;
                    
                    if (fsm.isNotRepeatState()){
                        target_speed = fsm.ble_command.value;
                        char telemetry[] = "testingSpeed";
                        pc.sendTelemetry(telemetry);
                        mdb.resetEncoders();
                    }

                    mdb.SetPwmFromTargetSpeed(dt,target_speed, target_speed);
                    if (fsm.shouldPrint()){
                        char telemetry[telemetry_size];
                        float speeds[2];
                        mdb.getSpeeds(speeds);

                        snprintf(telemetry, telemetry_size,
                        ",%.2f,%.4f,%.4f,%.4f,%.4f\r\n",
                        target_speed,speeds[0],speeds[1],mdb.left_motor.PWM_duty,mdb.right_motor.PWM_duty);        
                        
                        pc.sendTelemetry(telemetry);
                    }
                }
                break;

                default:
                    fsm.nextState(STATE_INVALID);
                    break;       
            }
        }
   
    }

}
