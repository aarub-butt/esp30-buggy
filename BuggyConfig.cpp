#include "BuggyConfig.hpp"

BuggyConfig::MotorPins BuggyConfig::left_motor_pins = 
{D1,D2,D4,D5};
BuggyConfig::MotorPins BuggyConfig::right_motor_pins = 
{D1,D2,D4,D5};

BuggyConfig::SensorPins BuggyConfig::sensor_pins = 
{PA_1,PA_2,PA_3,PA_4,PA_5,PA_6};
BuggyConfig::SensorWeights BuggyConfig::sensor_weights =
{0.1f,0.2f,0.3f,0.4f,0.5f,0.6f}; 