#include "reading-sensors.hpp"

void ReadingSensors(SensorBoard* sensor_board, ble* pc, FSM *fsm){
    if (fsm->isNotRepeatState()){
        char telemetry[] = "s:s1,s2,s3,s4,s5,s6\r\n";
        pc->sendTelemetry(telemetry);
    }

    if (fsm->shouldPrint()){
        float line_sensor_outputs[6];
        sensor_board->readSensorValues(line_sensor_outputs);

        int output_string_size = 64;

        char line_sensor_outputs_string[output_string_size];
        int int_line_sensor_outputs[6];
        for (int i = 0; i < 6; i++){
            int_line_sensor_outputs[i] = (int) (line_sensor_outputs[i] * 100);
        }

        snprintf(line_sensor_outputs_string,output_string_size,
        "s:%d,%d,%d,%d,%d,%d\r\n"
        ,int_line_sensor_outputs[0],int_line_sensor_outputs[1],int_line_sensor_outputs[2],
        int_line_sensor_outputs[3],int_line_sensor_outputs[4],int_line_sensor_outputs[5]);

        pc->sendTelemetry(line_sensor_outputs_string, fsm->global_timer.elapsed_time().count(), &fsm->cycle_timestamp);
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

        mdb->getSpeeds(getTimeElapsed_us(fsm->global_timer.elapsed_time().count(),&mdb->times), speeds);
        mdb->getPulseCounts(pulse_counts);

        int int_speeds[2];
        int_speeds[0] = (int) (speeds[0] *1000);
        int_speeds[1] = (int) (speeds[1] *1000);

        snprintf(telemetry, 64,
        "%d,%d,%d,%d\r\n",
        pulse_counts[0],int_speeds[0],
        pulse_counts[1], int_speeds[1]);
        
        pc->sendTelemetry(telemetry, fsm->global_timer.elapsed_time().count(), &fsm->cycle_timestamp);
    }

    fsm->nextState(fsm->getProgramState());
}

