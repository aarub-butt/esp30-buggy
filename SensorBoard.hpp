#pragma once

#include <mbed.h>
#include "BuggyConfig.hpp"

/**
* @file SensorBoard.hpp
* @brief Encapsulate logic for reading sensor data and calculating line position
*/

/**
* @class SensorBoard
* @brief Read sensor board data and calculate line position for steering buggy
*/
class SensorBoard{
    
    private:
        /**
        * @class LineSensor 
        * @brief Class to encapsulate values and interfacing of with a single sensor
        */
        class LineSensor{
            public: 
                AnalogIn input; ///< Pin for reading sensor analog values
                float weight; ///< Weight of individual sensor for calculating line position
                static float alpha; ///< Constant for EMA filter reading sensor values

                float current_sensor_value; ///< Current filtered reading
                float previous_sensor_value; ///< Previous filtered reading for use in EMA filter
                float normalised; ///< Current normalised reading (0 being black, 1 being white)

                float black; ///< Calibrated value for black
                float white; ///< Calibrated value for white
                
                /**
                * @brief Read and filter sensor value.
                */
                float readRaw();
                /**
                * @brief Read normalised sensor value
                */
                float read();

                /**
                * @brief Constructor providing pin and weight to individual sensor
                */
                LineSensor(PinName pin, float w);
        };
        
    public:


        LineSensor sensors[6]; ///< Array of 6 Line sensors from sensor board
        
        /**
        * @brief Initailise 6 sensor sensor board with specified pins and weights
        */
        SensorBoard(SensorConfig sensor_config); 

        /**
        * @brief Returns @ref SensorBoard::LineSensor::readRaw() of all sensors on sensor board
        * @param[out] sensorValues Stores ref SensorBoard::LineSensor::readRaw() of all sensors
        */
        void readRawSensorValues(float* sensorValues);


        /**
        * @brief Returns @ref SensorBoard::LineSensor::read() of all sensors on sensor board
        * @param[out] sensorValues Stores ref SensorBoard::LineSensor::read() of all sensors
        */
        void readSensorValues(float* sensorValues);

        
        /**
        * @brief Calculates line position error to steer the buggy
        * @param[out] line_error Stores calculated line position error
        * @return True when valid line position found else false when e.g. all sensors read black
        */
        bool getLinePosition(float* line_error);
        

        /** 
        * @brief Reads values of sensors to set for each sensor their @ref LineSensor::black and @ref LineSensor::white
        */
        void calibrate();
};