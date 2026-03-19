#include "reading-sensors.hpp"

void ReadingSensors(SensorBoard* sensor_board, ble* pc, FSM *fsm){
    float line_sensor_outputs[6];
    sensor_board->readSensorValues(line_sensor_outputs);
    
    if (fsm->shouldPrint()){
        int output_string_size = 64;

        char line_sensor_outputs_string[output_string_size];
        int int_line_sensor_outputs[6];
        for (int i = 0; i < 6; i++){
            int_line_sensor_outputs[i] = (int) (line_sensor_outputs[i] * 100);
        }

        snprintf(line_sensor_outputs_string,output_string_size,
        "s:%d,%d,%d,%d,%d,%d"
        ,int_line_sensor_outputs[0],int_line_sensor_outputs[1],int_line_sensor_outputs[2],
        int_line_sensor_outputs[3],int_line_sensor_outputs[4],int_line_sensor_outputs[5]);

        pc->sendTelemetry(line_sensor_outputs_string, fsm);
    }

    if (pc->getCommand(&fsm->ble_command)){
        switch (fsm->ble_command.cmd){
            
            case (FSM::BLE_COMMAND::CMD_MENU) :
                fsm->nextState(BuggyConfig::STATE_MENU);
                return;
                break;

            default:
                break;
        }
    }
    fsm->nextState(fsm->getProgramState());
}

void ReadingEncoder(ble* pc, MotorDriveBoard* mdb, FSM *fsm){
    if (fsm->isNotRepeatState()){
        char telemetry[] = "lc,ls,rc,rs\r\n";
        pc->sendTelemetry(telemetry);
    }

  

    if (fsm->shouldPrint()){
        char telemetry[64];
        int pulse_counts[2];
        float speeds[2];

        mdb->calcSpeed(fsm);
        mdb->getSpeed(speeds);
        mdb->getPulseCounts(pulse_counts);

        int int_speeds[2];
        int_speeds[0] = (int) (speeds[0] *1000);
        int_speeds[1] = (int) (speeds[1] *1000);

        
        snprintf(telemetry, 64,
        "%d,%d,%d,%d\r\n",
        pulse_counts[0],int_speeds[0],
        pulse_counts[1], int_speeds[1]);
        
        pc->sendTelemetry(telemetry, fsm);
    }

    if (pc->getCommand(&fsm->ble_command)){
        switch (fsm->ble_command.cmd){
            
            case (FSM::BLE_COMMAND::CMD_MENU) :
                fsm->nextState(BuggyConfig::STATE_MENU);
                return;
                break;

            default:
                break;
        }
    }
    fsm->nextState(fsm->getProgramState());
}

void TestParts(ble* pc, MotorDriveBoard* mdb, FSM* fsm){
    static bool change;

    if (fsm->isNotRepeatState()){
        change = false;
        char telemetry[]= "lpwm,rpwm,e";
        pc->sendTelemetry(telemetry);
    }


    if (change == true){
        char telemetry[64];
        
        float PWMs[2]; 
        mdb->getPWM(PWMs);
        int int_PWMs[2] = {
        (int) (PWMs[0] * 100), 
        (int) (PWMs[1] * 100)};

        snprintf(telemetry,64,"%d,%d,%d",
        int_PWMs[0],int_PWMs[1],mdb->getEnable());

        pc->sendTelemetry(telemetry,fsm);
        change = false;
    }

    if (pc->getCommand(&fsm->ble_command)){
        switch (fsm->ble_command.cmd){
            
            case (FSM::BLE_COMMAND::CMD_MENU) :
                fsm->nextState(BuggyConfig::STATE_MENU);
                return;
                break;

            case(FSM::BLE_COMMAND::CMD_SET_LEFT_PWM): {
                float PWMs[2];
                mdb->getPWM(PWMs);
                mdb->setPWM(fsm->ble_command.value, PWMs[1]);
                change = true;
                break;
            }

            case(FSM::BLE_COMMAND::CMD_SET_RIGHT_PWM):{
                float PWMs[2];
                mdb->getPWM(PWMs);
                mdb->setPWM(PWMs[0],fsm->ble_command.value);
                change = true;
                break;
            }

            case(FSM::BLE_COMMAND::CMD_ENABLE):{
                mdb->setEnable(!mdb->getEnable());
                change = true;
                break;
            }

            default:
                break;
        }
    }

    fsm->nextState(fsm->getProgramState());
}