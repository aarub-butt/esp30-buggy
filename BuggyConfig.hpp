#pragma once
#include <mbed.h>
/**
* @file BuggyConfig.hpp
* @brief Conatins various things for buggy to make main.cpp clean
*/

/**
* @struct MotorConfig
* @brief Pins structure and initial PID constants for each motor
*/
struct MotorConfig{
    PinName isBipolar; ///< Pin controlling whether in bipolar or unipolar mode
    PinName pwm; ///< Pin controlling PWM duty cycle
    PinName channel_A; ///< Pin for QED encoder channel A
    PinName channel_B; ///< Pinf or QED encoder channel B
    float kp; ///< Proportional constant for PID
    float ki; ///< Integral constant for PID
    float kd; ///< Derivative constant for PID
    float lim; ///< Output limit for PID
};


/**
* @struct SensorConfig
* @brief Configuration for sensor board array
*/
struct SensorConfig{

    /**
    * @struct Pin
    * @brief Pin for each sensor and weight for their values in calculating @ref getLinePosition()
    */
    struct Pin{
        PinName pin; ///< Pin for sensor to read Analog values
        float weight; ///< Weight for sensor in calculating line position
    };
    Pin sensors[6];  ///< Array containing configuration (@ref Pin) for 6 sensors for sensor board
};

#include "SensorBoard.hpp"

/**
* @enum ProgramState
* @brief Containing all states of program for @ref FSM
*/
enum ProgramState{
    STATE_INVALID = -1,
    STATE_STOP = 0,
    STATE_NONE = 1,
    STATE_SENSOR,
    STATE_ENCODER,
    STATE_ROTATE,
    STATE_DISPLAY,
    STATE_CALIBRATE,
    STATE_LINE_FOLLOWING,
    STATE_SPEED,
    STATE_TEST_SPEED
};

extern SensorConfig sensor_pins; ///< Sensor Pins preconfigured 
extern MotorConfig left_motor_pins; ///<  Left Motor Pins preconfigured
extern MotorConfig right_motor_pins; ///< Right Motor Pins preconfigured

/**
* @struct diff_time
* @brief Storing previous and current time to calculate elapsed time
*/
struct diff_time{
        long long previous_time; ///< Previous time
        long long current_time; ///< Current Time
        diff_time(); ///< Constructor setting values of @ref diff_time::previous_time and @ref diff_time::current_time to 0
};

static const int telemetry_size = 64; ///< Size of telemetry char[] to send via ble

/**
* @class FSM
* @brief Manage states of buggy and period of buggy cycles
*/
class FSM{
    private:
        ProgramState programState; ///< Current State
        ProgramState previousProgramState; ///< Previous state to detect whether state has changed
        static constexpr auto buggy_period = std::chrono::microseconds(6500); ///< Cycle period of buggy at 4.5ms

        Ticker buggy_rate; ///< Hardware interrupt to trigger cycle using @ref InitiateCycle()
        void InitiateCycle(); ///< ISR to set @ref var_isNextCycle
        volatile bool var_isNextCycle; ///< Flag true when buggy in next cycle set by @ref InitiateCycle()
        int var_shouldPrint;  ///< Counter to decrease rate to send telemetry
        bool should_send_telemetry; ///< Flag set by @ref toggleTelemetry() whether to spend time sending Telemetry

    public:
        /**
        * @brief Constructor initialised @ref programState and @ref buggy_rate and others to start system
        */
        FSM(); 

        /**
        * @brief Check if @ref var_isNextCycle is true to start cycle
        * @return Return true when start cycle 
        */
        bool isNextCycle(); 

        static Timer global_timer; ///< Single global timer in us for all buggy system. 
        diff_time cycle_timestamp; ///< Timestamp for how long a cycle takes

        ProgramState return_state; ///< State to return to after temporary state change

        /**
        * @brief Check conditions if should send telemtry 
        * - @ref var_shouldPrint >= 20
        * - @ref should_send_telemetry is True
        * @return True when both conditions met else false
        */
        bool shouldPrint();

        /**
        * @brief Toggle @should_send_telemetry to send telemetry or not
        */
        void toggleTelemetry();

        /**
        * @brief Update @ref cycle_timestamp with current @ref global_timer value
        */
        void start_timestamp();

        /**
        * @brief Check whether new to state, used for initaliasing values when entering state
        * @return False when @ref previousProgramState is @ref programState , else false 
        * @note Sets @ref previousProgramState to @ref programState if true
        */
        bool isNotRepeatState();

        /**
        * @brief Move on to next state
        * @param nextState @ref ProgramState to move @ref programState to
        * @note @ref previousProgramState equals old @ref programState
        */
        void nextState(ProgramState nextState);

        /**
        * @brief Returns current state of buggy
        * @return Current state @ref programState e.g. (STATE_NONE, STATE_LINE)
        */
        ProgramState getProgramState();

        /**
        * @brief Returns previous state of buggy
        * @return Previous state @ref previousProgramState e.g. (STATE_NONE, STATE_LINE)
        */
        ProgramState getPreviousProgramState();

        /**
        * @class BLE_COMMAND
        * @brief Class to store parsed command sent via BLE
        */
        class BLE_COMMAND{
            public:
                char command[telemetry_size]; ///< String of command e.g. pwm=0.5,0.5
                float value; ///< Value of command e.g. PWM duty of 0.5

                void clear(); ///< Clear command and value
                BLE_COMMAND(); 
        };
        BLE_COMMAND ble_command; ///< Commmand received via BLE
};


/**
 * @brief Calculates elapsed time in microseconds.
 * @param[in] current_time Current time reading
 * @param[in,out] times Pointer to @ref diff_time struct to update previous/current time values
 * @param[out] dt_us Pointer to store elapsed time in microseconds
 */
void getTimeElapsed(long long current_time, diff_time *times, int* dt_us);

/**
 * @brief Calculates elapsed time in seconds (float).
 * @param[in] current_time Current time reading
 * @param[in,out] times Pointer to @ref diff_time struct to update previous/current time values
 * @param[out] dt_s Pointer to store elapsed time in seconds
 */
void getTimeElapsed(long long current_time, diff_time *times, float* dt_s);


