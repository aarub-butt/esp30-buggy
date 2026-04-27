#pragma once

/**
* @file MotorDriveBoard.hpp
* @brief Encapsulates the MotorDriveBoard class for PID control, 
* controlling motors and reading encoders of the motors for the line-following buggy.
*/

#include <mbed.h>
#include "QEI.h"
#include "BuggyConfig.hpp"

/**
* @class MotorDriveBoard
* @brief Class which encapsulates control of motors.
** Containes pid control loops with use of @ref PID_controller
** Motor hardware control with @ref Motor class, 
** Reading of encoder sensors with use of imported @ref QEI class.
*/
class MotorDriveBoard{
    public:

        /**
        * @class PID_controller
        * @brief Resuable code for PID control loops, can zero out either kp,ki,kd constants for PI control etc.
        */
        class PID_controller{
            public:
                float kp; ///< Proportional control constant, response to current error.
                float ki; ///< Integral control constant, response to accumulated error.
                float kd; ///< Derivative control constant, response to change in error error.
                float previous_error; ///< Previous error to calculate change in error for derivative control.
                float output_limit; ///< Maximum output for PID control loop, e.g. 0.5f for motor bipolar PWM duty cycle.
                float integral; ///< Accumlation of past error for integral control.

                /**
                * @brief Sets integral and previous error to 0
                */
                void reset();
                
                /**
                * @brief Sets constants for PID control
                * @param p sets @ref kp constant
                * @param i sets @ref ki constant
                * @param d sets @ref kd constant
                */
                void setPid(float p , float i , float d);

                /**
                * @brief Calculate PID control output
                * @param error Difference between target and currrent value
                * @param dt Time elapsed since previous calculation
                * @return Output of PID control, clamped with @ref output_limit
                */
                float calculate(float error, float dt);


                /**
                * @brief Constructor for @ref PID_controller class
                * @param p sets @ref kp constant
                * @param i sets @ref ki constant
                * @param d sets @ref kd constant
                * @param lim sets @ref output_limit of PID control
                * @note Sets @ref previous_error and @ref integral to 0
                */
                PID_controller(float p,float i,float d, float lim);
        };

    
    private:

        float PWM_frequency; ///< Frequency of PWM into @ref Motor
        DigitalOut enable; ///< Enable pin for buggy. Controling whether PWM output into @ref Motor
        static const float wheel_track_length; ///< Constant for distance between 2 wheels of buggy for @ref MotorDriveBoard::startRotate()
        static float alpha; ///< EMA filter constant for reading encoder values in @ref MotorDriveBoard::Motor::calcSpeed()

        /**
        * @class Motor
        * @brief Encapsulates code for direct hardware control of motors, and reading of their respective encoder
        */
        class Motor{
            public:

                PID_controller speed_pid; ///< PID control loop for calculating correct PWM output for target speed 

                // encoder variables

                int previous_pulse_count; ///< Previous pulse count read by @ref encoder
                int current_pulse_count; ///< Pulse count read by @ref encoder
                float distance_travelled; ///< Distance travelled from previous calculation using @ref previous_pulse_count and @ref current_pulse_count

                // motor variables
                float speed; ///< Current speed read from @ref encoder for motor
                float previous_speed; ///< Previous speed read from @ref encoder for motor
                float speed_error; ///< Difference between @ref speed and @ref previous_speed
                float PWM_duty; ///< Mirror of PWM duty cycle currently written to @ref PWM


                // conastants
                static const int pulses_per_revolution; ///< Constant for pulse count read by @encoder for 1 revolution of wheel connected to bguggy
                static const float wheel_circumference; ///< Constant for circumference of wheel connected to motor

                // Physical Pins
                DigitalOut isBipolar; ///< Controlling whether motor is in Bipolar or Unipolar mode
                PwmOut PWM; ///< Controlling PWM duty cycle into motor
                QEI encoder; ///< Reading speed of motor and distance travelled by motor

                /**
                * @brief Reset encoder values 
                *
                * Resets @ref encoder object 
                *
                * And set @ref distance_travelled @ref previous_pulse_count @ref current_pulse_count to 0
                */
                void resetEncoder();

                /** 
                * @brief Calculate speed of motor from previous time calculated speed 
                * @param time_elapsed is time since previous time calculated speed
                */
                void calcSpeed(float time_elapsed);
                
                /**
                * @brief Calculate distance travelled by wheel since previous time calculated 
                */
                void calcDistanceTravelled();

                /**
                * @brief Constructor for @Motor
                * @param motor_config uses @ref MotorConfig struct for storing pins and constants for constructing motor.
                */
                Motor(MotorConfig motor_config);                
        };

        int _target_pulse_count; ///< Varaible for target pulse count for rotation

    public:
        static float max_speed; ///< Speed limit of buggy @ref Motor

        /**
        * @brief Update @ref MotorDriveBoard::Motor::Speed and @ref MotorDriveBoard::Motor::encoder values for @ref MotorDriveBoard::Motor
        * @param dt Time since previous call of function
        */
        void updateSpeeds(float dt);

        Motor left_motor; ///< Encapsulating control for left motor
        Motor right_motor; ///< Encapsulating control for right motor
        diff_time times; ///< Using @ref diff_time contains previous and current time since @ref updateSpeeds() called

        static PID_controller steering_pid; ///< PID controller for steering buggy for line following
        static float dynamic_speed_constant; ///< Constant for adjusting base speed, faster on straight sections, slower on bends


        /**
        * @brief Constructor for @ref MotorDriveBoard
        * @param left_motor_config using @ref MotorConfig struct containing pins for left motor
        * @param right_motor_config using @ref MotorConfig struct containing pins for right motor
        * @param e Pin for enable pin
        * @param pwmFrequency Frequency for @ref left_motor and @ref right_motor PWM duty cycle to run at 
        */
        MotorDriveBoard(MotorConfig left_motor_config, MotorConfig right_motor_config, PinName e, float pwmFrequency);
        
        /**
        * @brief Set @ref enable to 1 or 0 so @ref MotorDriveBoard::Motor get PWM output or not respectively
        * @param isEnable Value to set @ref enable 
        */
        void setEnable(bool isEnable);
        
        /** 
        * @brief Return value of @ref enable
        * @return Enable pin value
        */
        bool getEnable();

        /**
        * @brief Set PWM duty cycle of @ref left_motor and @ref right_motor
        * @param left Duty cycle for @ref left_motor
        * @param right Duty cycle for @ref right_motor
        */
        void setPWM(float left, float right);

        /**
        * @brief Returns @ref MotorDriveBoard::Motor::PWM_duty of both motors
        * @param[out] PWMs Stores @ref MotorDriveBoard::Motor::PWM_duty of [@ref left_motor @ref right_motor]
        */
        void getPWM(float* PWMs);

        /**
        * @brief Returns @ref MotorDriveBoard::Motor:speed of both motors
        * @param[out] speeds Stores @ref MotorDriveBoard::Motor::speed of @ref left_motor in [0] and @ref right_motor in [1]
        */
        void getSpeeds(float* speeds);

        /**
        * @brief Returns @ref MotorDriveBoard::Motor::current_pulse_count of both motors
        * @param[out] encoder_values Stores @ref MotorDriveBoard::Motor::current_pulse_count of @ref left_motor in [0] and @ref right_motor in [1]
        */
        void getPulseCounts(int* enocder_values);
        
        /**
        * @brief Resets encoders of @ref left_motor and @ref right_motor
        */
        void resetEncoders();

        /**
        * @brief Using target speeds calcuates PWM duty cycles using @ref MotorDriveBoard::Motor::speed_pid for @ref left_motor and @ref right_motor
        * @param Time since previous cycle 
        * @param lt @ref left_motor target speed
        * @param rt @ref right_motor target speed
        */
        void SetPwmFromTargetSpeed(float dt, float lt, float rt);

        /**
        * @brief Set PWM for @ref left_motor and @ref right_motor so buggy follows the line
        * @param error How far away buggy is from following the line
        * @param dt Time since previous cycle 
        */
        void updateLineFollower(float error, float dt);


        /** 
        * @brief Set @ref _is_rotating to true and calculate @ref _target_pulse_count to end rotating
        * @param degree Amount of degree to rotate. Positive values rotating clockwise, negative values rotating anti-clockwise
        */
        void startRotate(float degree); 
        /**
        * @brief Cycle through stages of rotation stop -> rotate -> stop and only rotate when @ref _is_rotating is true
        * @ref dt Time elapsed since previous call of @ref updateRotate()
        * @return Return true when rotation is finished
        */
        bool updateRotate(float dt);

        /**
        * @brief Calculate PWM duty cycle to gradually stop buggy
        * @param dt Time since previous call
        * @return Return true when buggy has stopped
        */
        bool stop(float dt);
};