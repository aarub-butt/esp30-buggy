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
                float normalised;

                float black;
                float white;
                
                float readRaw();
                float read();
                LineSensor(PinName pin, float w);
        };
        
    public:
        LineSensor sensors[6];
        SensorBoard(SensorConfig sensor_config);

        void readRawSensorValues(float* sensorValues);
        void readSensorValues(float* sensorValues);
        bool getLinePosition(float* line_error, long long current_time);
        void calibrate();

};