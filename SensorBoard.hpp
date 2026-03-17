#pragma once

#include <mbed.h>
#include "BuggyConfig.hpp"

class SensorBoard{
    private:
        struct LineSensor{
            AnalogIn input;
            float weight;

            LineSensor(PinName pin, float w) : input(pin), weight(w){}
        };
        
        LineSensor sensors[6];
    public:
        SensorBoard(BuggyConfig::SensorPins sensor_pins, BuggyConfig::SensorWeights sensor_weights);

        void readSensorValues(float* sensorValues);
        float getLinePosition();

};