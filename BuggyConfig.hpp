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
        STATE_NONE = -1,
        STATE_MENU = 0,
        STATE_SENSOR,
        STATE_ENCODER,
        STATE_TEST
    };

    extern SensorWeights sensor_weights;
    extern SensorPins sensor_pins;
    extern MotorPins left_motor_pins;
    extern MotorPins right_motor_pins;

    struct diff_time{
            int previous_time;
            int current_time;
            diff_time();
    };

}



class FSM{
    private:
        BuggyConfig::ProgramState programState;
        BuggyConfig::ProgramState previousProgramState;
        static constexpr auto buggy_period = std::chrono::microseconds(4500); 

        Ticker buggy_rate;
        void InitiateCycle();
        bool var_isNextCycle;
        int send_telemetry_cycle_time;

        int var_shouldPrint;

        

    public:
        FSM();
        bool isNextCycle();
        static Timer global_timer;
        BuggyConfig::diff_time cycle_timestamp;

        bool shouldPrint();
        void start_timestamp();

        bool isNotRepeatState();
        void nextState(BuggyConfig::ProgramState nextState);
        BuggyConfig::ProgramState getProgramState();

        


        class BLE_COMMAND{
            public:
                enum CMD_INDEX{
                    CMD_INVALID = -1,
                    CMD_MENU = 0,
                    CMD_ENABLE,
                    CMD_SET_LEFT_PWM,
                    CMD_SET_RIGHT_PWM,
                    CMD_TEST,
                    CMD_ENCODER,
                    CMD_SENSOR,
                };

                char command[64];
                float value;
                CMD_INDEX cmd;

                void clear();
                BLE_COMMAND();
        };
        BLE_COMMAND ble_command;
};

int getTimeElapsed_us(FSM* fsm, BuggyConfig::diff_time *times);

