#pragma once

#include <mbed.h>
#include "QEI.h"
#include "BuggyConfig.hpp"

class MotorDriveBoard{
    private:
        float PWM_frequency;
        DigitalOut enable;
        static const float wheel_track_length;
                
        // Timer variables
        int previous_time;
        int current_time;

        class Motor{
            private:

            public:
                // encoder variables
                int previous_pulse_count;
                int current_pulse_count;
                int distance_travelled;

                // motor variables
                float speed;
                float RPM;
                float PWM_duty;
                
                // conastants
                static const int pulses_per_revolution;
                static const float wheel_circumference;

                // Physical Pins
                DigitalOut isBipolar;
                PwmOut PWM;
                QEI encoder;

                void resetEncoder();
                
                Motor(BuggyConfig::MotorPins motor_pins);                
        };

        Motor left_motor;
        Motor right_motor;

        void updateEncoder();

    public:
        MotorDriveBoard(BuggyConfig::MotorPins left_motor_pins, BuggyConfig::MotorPins right_motor_pins, PinName e, float pwmFrequency);
        

        void setEnable(bool isEnable);
        bool getEnable();
        void setIsBipolar(bool isBipolar);
        void set_PWM_frequency(float PWM_frequency);
        void setPWM(float left, float right);
        void getPWM(float* PWMs);

        void getSpeed(float* speeds);
        void calcSpeed(Timer* timer_us);

        void getCurrentEncoder(int* enocder_values);
        void resetEncoders();
};