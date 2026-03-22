#pragma once
#include <mbed.h>

struct MotorConfig{
    PinName isBipolar;
    PinName pwm;
    PinName channel_A;
    PinName channel_B;
    float kp;
    float ki;
    float kd;
    float lim;
};

struct SensorConfig{
    struct Pin{
        PinName pin;
        float weight;
    };
    Pin sensors[6];
};



enum ProgramState{
    STATE_INVALID = -1,
    STATE_STOP = 0,
    STATE_NONE = 1,
    STATE_SENSOR,
    STATE_ENCODER,
    STATE_ROTATE,
    STATE_DISPLAY,
    STATE_TELEMETRY
};

extern SensorConfig sensor_pins;
extern MotorConfig left_motor_pins;
extern MotorConfig right_motor_pins;

struct diff_time{
        int previous_time;
        int current_time;
        diff_time();
};



class FSM{
    private:
        ProgramState programState;
        ProgramState previousProgramState;
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
        diff_time cycle_timestamp;

        bool shouldPrint();
        void start_timestamp();

        bool isNotRepeatState();
        void nextState(ProgramState nextState);
        ProgramState getProgramState();
        ProgramState getPreviousProgramState();

        class BLE_COMMAND{
            public:
                char command[64];
                float value;

                void clear();
                BLE_COMMAND();
        };
        BLE_COMMAND ble_command;
};

int getTimeElapsed_us(long long current_time, diff_time *times);

