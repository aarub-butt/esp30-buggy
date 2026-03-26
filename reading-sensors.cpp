#include "reading-sensors.hpp"
#include "BuggyConfig.hpp"

void ReadingSensors(SensorBoard* sensor_board, ble* pc, FSM *fsm){
    if (fsm->isNotRepeatState()){
        char telemetry[] = "s:s1,s2,s3,s4,s5,s6\r\n";
        pc->sendTelemetry(telemetry);
    }

    if (fsm->shouldPrint()){
        float line_sensor_outputs[6];
        sensor_board->readSensorValues(line_sensor_outputs);
        char line_sensor_outputs_string[telemetry_size];
 
        snprintf(line_sensor_outputs_string,telemetry_size,
        "s:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n"
        ,line_sensor_outputs[0],line_sensor_outputs[1],line_sensor_outputs[2],
        line_sensor_outputs[3],line_sensor_outputs[4],line_sensor_outputs[5]);

        pc->sendTelemetry(line_sensor_outputs_string, fsm->global_timer.elapsed_time().count(), &fsm->cycle_timestamp);
    }

}

void ReadingEncoder(ble* pc, MotorDriveBoard* mdb, FSM *fsm){
    if (fsm->isNotRepeatState()){
        char telemetry[] = "lc,ls,rc,rs\r\n";
        pc->sendTelemetry(telemetry);
        mdb->resetEncoders();
    }

    if (fsm->shouldPrint()){
        char telemetry[telemetry_size];
        int pulse_counts[2];
        float speeds[2];

        mdb->getSpeeds(speeds);
        mdb->getPulseCounts(pulse_counts);
        
        snprintf(telemetry, telemetry_size,
        "%d,%.2f,%d,%.2f\r\n",
        pulse_counts[0],speeds[0],
        pulse_counts[1], speeds[1]);
        
        /*
        snprintf(telemetry, telemetry_size,
        "%.4f,%.4f\r\n",
        speeds[0],speeds[1]);   */     
        
        pc->sendTelemetry(telemetry);
        //pc->sendTelemetry(telemetry, fsm->global_timer.elapsed_time().count(), &fsm->cycle_timestamp);
    }

}

