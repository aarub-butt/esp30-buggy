#include "SensorBoard.hpp"

// Constants


// LineSensor Methods

float SensorBoard::LineSensor::read(){
    current_sensor_value = (alpha * input.read()) + ((1-alpha) * previous_sensor_value);
    previous_sensor_value = current_sensor_value;
    return current_sensor_value;
}

// SensorBoard Methods

void SensorBoard::readSensorValues(float* sensorValues){ 
    for (int i = 0; i < 6; i++){
        sensorValues[i] = sensors[i].read();
    }
    return;
}

float SensorBoard::getLinePosition(){
    float weightedSum = 0.0f;
    float totalReading = 0.0f;

    for (int i = 0; i<6 ; i++){
        float sensorValue = sensors[i].read();
        totalReading += sensorValue;
        weightedSum += sensorValue * sensors[i].weight;
    }

    return (weightedSum / totalReading);
}

// Constructors
SensorBoard::LineSensor::LineSensor(PinName pin, float w) : input(pin), weight(w){
    current_sensor_value = 0;
    previous_sensor_value = 0;
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