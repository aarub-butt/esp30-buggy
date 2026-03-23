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

#include "SensorBoard.hpp"


enum ProgramState{
    STATE_INVALID = -1,
    STATE_STOP = 0,
    STATE_NONE = 1,
    STATE_SENSOR,
    STATE_ENCODER,
    STATE_ROTATE,
    STATE_DISPLAY,
    STATE_CALIBRATE,
    STATE_LINE_FOLLOWING
};

extern SensorConfig sensor_pins;
extern MotorConfig left_motor_pins;
extern MotorConfig right_motor_pins;

struct diff_time{
        long long previous_time;
        long long current_time;
        diff_time();
};
static const int telemetry_size = 64;

class FSM{
    private:
        ProgramState programState;
        ProgramState previousProgramState;
        static constexpr auto buggy_period = std::chrono::microseconds(4500); 

        Ticker buggy_rate;
        void InitiateCycle();
        bool var_isNextCycle;
        int var_shouldPrint; 
        bool should_send_telemetry;

    public:
        FSM();
        bool isNextCycle();
        static Timer global_timer;
        diff_time cycle_timestamp;

        ProgramState return_state;

        bool shouldPrint();
        void toggleTelemetry();
        void start_timestamp();

        bool isNotRepeatState();
        void nextState(ProgramState nextState);
        ProgramState getProgramState();
        ProgramState getPreviousProgramState();

        class BLE_COMMAND{
            public:
                char command[telemetry_size];
                float value;

                void clear();
                BLE_COMMAND();
        };
        BLE_COMMAND ble_command;
};

void getTimeElapsed(long long current_time, diff_time *times, int* dt_us);
void getTimeElapsed(long long current_time, diff_time *times, float* dt_s);


