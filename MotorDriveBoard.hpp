#pragma once

#include <mbed.h>
#include "QEI.h"
#include "BuggyConfig.hpp"

class MotorDriveBoard{
    public:
        class PID_controller{
            public:
                float kp;
                float ki;
                float kd;
                float previous_error;
                float output_limit;
                float integral;

                void reset();
                void setPid(float p , float i , float d);

                float calculate(float error, float dt);

                PID_controller(float p,float i,float d, float lim);
        };

    
    private:
        float PWM_frequency;
        DigitalOut enable;
        static const float wheel_track_length;
        static float alpha;
        static float max_speed;

        // Timer variables

        class Motor{
            public:

                PID_controller speed_pid;

                // encoder variables

                int previous_pulse_count;
                int current_pulse_count;
                float distance_travelled;

                // motor variables
                float speed;
                float previous_speed;
                float speed_error;
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
                void calcSpeed(float time_elapsed);
                void calcDistanceTravelled();

                void setSpeed(float error);

                Motor(MotorConfig motor_config);                
        };



        void updateEncoder();

        int _target_pulse_count;
        int _rotation_stage;
        bool rotate(float dt);

    public:
        void updateSpeeds(float dt);

        static PID_controller rotation_pid;


        Motor left_motor;
        Motor right_motor;
        diff_time times;

        static PID_controller steering_pid;
        static float dynamic_speed_constant;

        MotorDriveBoard(MotorConfig left_motor_config, MotorConfig right_motor_config, PinName e, float pwmFrequency);
        
        void setEnable(bool isEnable);
        bool getEnable();
        void setPWM(float left, float right);
        void getPWM(float* PWMs);
        void getSpeeds(float* speeds);
        void getPulseCounts(int* enocder_values);
        void resetEncoders();

        void SetPwmFromTargetSpeed(float dt, float lt, float rt);
        void updateLineFollower(float error, float dt);

        bool _is_rotating;
        void startRotate(float degree);
        bool updateRotate(float dt);

        bool stop(float dt);
};