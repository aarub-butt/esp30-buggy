#include "reading-sensors.hpp"


void ReadingSensors(SensorBoard* sensor_board, ble* pc){
    float line_sensor_outputs[6];
    sensor_board->readSensorValues(line_sensor_outputs);
    
    int output_string_size = 64;
    char line_sensor_outputs_string[output_string_size];
    snprintf(line_sensor_outputs_string,output_string_size,
    "s:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f"
    ,line_sensor_outputs[0],line_sensor_outputs[1],line_sensor_outputs[2],
    line_sensor_outputs[3],line_sensor_outputs[4],line_sensor_outputs[5]);

    pc->sendTelemetry(line_sensor_outputs_string);
}

void ReadingEncoder(ble* pc, MotorDriveBoard* mdb){
    
}