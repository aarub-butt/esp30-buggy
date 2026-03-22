#pragma once

#include <mbed.h>
#include "BuggyConfig.hpp"

class SensorBoard{
    
    private:
        class LineSensor{
            public: 
                AnalogIn input;
                float weight;

                static float alpha;

                float current_sensor_value;
                float previous_sensor_value;
                

                float read();
                LineSensor(PinName pin, float w);
        };
        
        LineSensor sensors[6];
    
    public:
        SensorBoard(SensorConfig sensor_config);

        void readSensorValues(float* sensorValues);
        float getLinePosition();
};