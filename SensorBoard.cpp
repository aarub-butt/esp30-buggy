#include "SensorBoard.hpp"

// SensorBoard Methods

void SensorBoard::readSensorValues(float* sensorValues){ 
    for (int i = 0; i < 6; i++){
        sensorValues[i] = sensors[i].input.read();
    }
    return;
}

float SensorBoard::getLinePosition(){
    float weightedSum = 0.0f;
    float totalReading = 0.0f;

    for (int i = 0; i<6 ; i++){
        float sensorValue = sensors[i].input.read();
        totalReading += sensorValue;
        weightedSum += sensorValue * sensors[i].weight;
    }

    return (weightedSum / totalReading);
}

// Constructors

SensorBoard::SensorBoard(BuggyConfig::SensorPins sensor_pins, BuggyConfig::SensorWeights sensor_weights) : 
sensors{
    {sensor_pins.s1 , sensor_weights.s1},
    {sensor_pins.s2 , sensor_weights.s2},
    {sensor_pins.s3 , sensor_weights.s3},
    {sensor_pins.s4 , sensor_weights.s4},
    {sensor_pins.s5 , sensor_weights.s5},
    {sensor_pins.s6 , sensor_weights.s6},
}
{}