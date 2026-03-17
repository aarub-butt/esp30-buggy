#pragma once
#include <mbed.h>

namespace BuggyConfig{
    struct MotorPins{
        PinName isBipolar;
        PinName pwm;
        PinName channel_A;
        PinName channel_B;
    };

    struct SensorPins{
        PinName s1;
        PinName s2;
        PinName s3;
        PinName s4;
        PinName s5;
        PinName s6;
    };

    struct SensorWeights{
        float s1;
        float s2;
        float s3;
        float s4;
        float s5;
        float s6;
    };

    enum ProgramState{
        STATE_INITAL = 0,
        STATE_SENSOR_OUTPUT,
        STATE_LINE_POSITION,
        STATE_ENCODER_SPEED,
        STATE_LINE_FOLLOWING
    };

    extern SensorWeights sensor_weights;
    extern SensorPins sensor_pins;
    extern MotorPins left_motor_pins;
    extern MotorPins right_motor_pins;
}
