#include "SensorBoard.hpp"

// LineSensor Methods

float SensorBoard::LineSensor::readRaw(){
    current_sensor_value = (alpha * input.read()) + ((1-alpha) * previous_sensor_value);
    previous_sensor_value = current_sensor_value;
    return current_sensor_value;
}

float SensorBoard::LineSensor::read(){
    readRaw();
    normalised = 1.0f- ((current_sensor_value-white)/(black-white));
    if (normalised > 1.0f) normalised = 1.0f;
    if (normalised < 0.0f) normalised = 0.0f;
    return normalised;
}

// SensorBoard Methods

void SensorBoard::readSensorValues(float* sensorValues){
    for  (int i = 0; i < 6; i++){
        sensorValues[i] = sensors[i].read();
    }
}

bool SensorBoard::getLinePosition(float* line_error, long long current_time){
    float weighted_sum = 0.0f;
    float total_reading = 0.0f;

    for (int i = 0; i<6 ; i++){
        float sensor_value = sensors[i].read();
        total_reading += sensor_value;
        weighted_sum += sensor_value * sensors[i].weight;
    }

    if (total_reading > 0.5){
        *line_error = weighted_sum/total_reading;
        return true;
    }else{
        return false;
    }
}

void SensorBoard::calibrate(){
    for (int i = 0; i < 6; i++){
        float sensor_value = sensors[i].readRaw();
        float* black = &sensors[i].black;
        float* white = &sensors[i].white;
        if (sensor_value > *black) *black = sensor_value;
        if (sensor_value < *white) *white = sensor_value;  
    }
}

// Constructors
SensorBoard::LineSensor::LineSensor(PinName pin, float w) : input(pin), weight(w){
    current_sensor_value = 0.5f;
    previous_sensor_value = 0.5f;

    black = 0.8f;
    white = 0.2f;
}

SensorBoard::SensorBoard(SensorConfig sensor_config) : 
sensors{
    {sensor_config.sensors[0].pin , sensor_config.sensors[0].weight},
    {sensor_config.sensors[1].pin , sensor_config.sensors[1].weight},
    {sensor_config.sensors[2].pin , sensor_config.sensors[2].weight},
    {sensor_config.sensors[3].pin , sensor_config.sensors[3].weight},    
    {sensor_config.sensors[4].pin , sensor_config.sensors[4].weight},
    {sensor_config.sensors[5].pin , sensor_config.sensors[5].weight}
}{}