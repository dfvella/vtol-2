#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <Arduino.h>

#include "imu_driver.h"
#include "pwm_schedular.h"
#include "pid_controller.h"

// forward control map gains
#define FORWARD_YAW_DIFFERENTIAL 0.5
#define FORWARD_RIGHT_TILT 940
#define FORWARD_LEFT_TILT 2120

// slow control map gains
#define SLOW_YAW_DIFFERENTIAL 0.1
#define SLOW_ROLL_DIFFERENTIAL 0.1
#define SLOW_FLAPS_TRIM -100
#define SLOW_RIGHT_TILT 1500
#define SLOW_LEFT_TILT 1570

// vertical control map gains
#define VERTICAL_ROLL_DIFFERENTIAL 0.3
#define VERTICAL_YAW_MOTOR_TILT 0.5
#define VERTICAL_PITCH_MOTOR_TILT 0.5
#define VERTICAL_FLAPS_TRIM -100
#define VERTICAL_RIGHT_TILT 1850
#define VERTICAL_LEFT_TILT 1260
#define VERTICAL_YAW_TRIM 40

// autolevel attitude limits
#define AUTO_MAX_ROLL_ANGLE 40
#define AUTO_MAX_PITCH_ANGLE 40
#define AUTO_MAX_YAW_RATE 2.5
#define AUTO_MAX_YAW_ERROR 30

// use different rates for different flight modes
#define RATE_MAX_ROLL_RATE 1.5 // units: deg per 20 ms
#define RATE_MAX_PITCH_RATE 1.0
#define RATE_MAX_YAW_RATE 2.5
#define RATE_MAX_ROLL_ERROR 30
#define RATE_MAX_PITCH_ERROR 30
#define RATE_MAX_YAW_ERROR 30

// for noise reduction
#define DEAD_STICK 20 // microseconds
#define NEUTRAL_STICK 1500
#define NEUTRAL_THROTTLE 1000

// vertical mode pid gains
#define ROLL_P_VERTICAL 10 // was 7
#define ROLL_I_VERTICAL 0
#define ROLL_D_VERTICAL 4 // was 3
#define ROLL_I_MAX_VERTICAL 1

#define PITCH_P_VERTICAL 11 // was 13
#define PITCH_I_VERTICAL 0
#define PITCH_D_VERTICAL 6 // was 4
#define PITCH_I_MAX_VERTICAL 1

#define YAW_P_VERTICAL 8 // was 6
#define YAW_I_VERTICAL 0
#define YAW_D_VERTICAL 3 // was 2
#define YAW_I_MAX_VERTICAL 1

// slow mode pid gains
#define ROLL_P_SLOW 4
#define ROLL_I_SLOW 0
#define ROLL_D_SLOW 0.5
#define ROLL_I_MAX_SLOW 1

#define PITCH_P_SLOW 20
#define PITCH_I_SLOW 0
#define PITCH_D_SLOW 0.5
#define PITCH_I_MAX_SLOW 70

#define YAW_P_SLOW 20
#define YAW_I_SLOW 0
#define YAW_D_SLOW 0.1
#define YAW_I_MAX_SLOW 1

// forward mode pid gains
#define ROLL_P_FORWARD 4
#define ROLL_I_FORWARD 0
#define ROLL_D_FORWARD 0.5
#define ROLL_I_MAX_FORWARD 1

#define PITCH_P_FORWARD 20
#define PITCH_I_FORWARD 0
#define PITCH_D_FORWARD 0.5
#define PITCH_I_MAX_FORWARD 70

#define YAW_P_FORWARD 20
#define YAW_I_FORWARD 0
#define YAW_D_FORWARD 0.1
#define YAW_I_MAX_FORWARD 1

// trim
#define ROLL_PID_TRIM 0
#define PITCH_PID_TRIM 0
#define YAW_PID_TRIM 0

// servo throw
#define MIN_ELEVATOR_THROW 1200
#define CENTER_ELEVATOR 1500
#define MAX_ELEVATOR_THROW 1800

#define MIN_RIGHT_ALR_THROW 1200
#define CENTER_RIGHT_ALR 1500
#define MAX_RIGHT_ALR_THROW 1800

#define MIN_LEFT_ALR_THROW 1200
#define CENTER_LEFT_ALR 1500
#define MAX_LEFT_ALR_THROW 1800

// flight mode transition
#define TILT_TRANSITION_DAMPER 10
#define TILT_TRANSITION_THRESHOLD 1700

// time spent in the transition state between flight mode changes
#define TRANSITION_TIME 50 // 20 millisecond units

// Three position switch thresholds
#define SWITCH_POS1 1300
#define SWITCH_POS2 1700

#define MIN_PWM_PULSEWIDTH 1000
#define MAX_PWM_PULSEWIDTH 2000

enum class Control_Mode : uint8_t { MANUAL, RATE, AUTOLEVEL };

const PIDcontroller::Gains FORWARD_ROLL_GAINS {
    ROLL_P_FORWARD, ROLL_I_FORWARD, ROLL_D_FORWARD, ROLL_I_MAX_FORWARD
};

const PIDcontroller::Gains SLOW_ROLL_GAINS {
    ROLL_P_SLOW, ROLL_I_SLOW, ROLL_D_SLOW, ROLL_I_MAX_SLOW
};

const PIDcontroller::Gains VERTICAL_ROLL_GAINS {
    ROLL_P_VERTICAL, ROLL_I_VERTICAL, ROLL_D_VERTICAL, ROLL_I_MAX_VERTICAL
};

const PIDcontroller::Gains FORWARD_PITCH_GAINS {
    PITCH_P_FORWARD, PITCH_I_FORWARD, PITCH_D_FORWARD, PITCH_I_MAX_FORWARD
};

const PIDcontroller::Gains SLOW_PITCH_GAINS {
    PITCH_P_SLOW, PITCH_I_SLOW, PITCH_D_SLOW, PITCH_I_MAX_SLOW
};

const PIDcontroller::Gains VERTICAL_PITCH_GAINS {
    PITCH_P_VERTICAL, PITCH_I_VERTICAL, PITCH_D_VERTICAL, PITCH_I_MAX_VERTICAL
};

const PIDcontroller::Gains FORWARD_YAW_GAINS {
    YAW_P_FORWARD, YAW_I_FORWARD, YAW_D_FORWARD, YAW_I_MAX_FORWARD
};

const PIDcontroller::Gains SLOW_YAW_GAINS {
    YAW_P_SLOW, YAW_I_SLOW, YAW_D_SLOW, YAW_I_MAX_SLOW
};

const PIDcontroller::Gains VERTICAL_YAW_GAINS {
    YAW_P_VERTICAL, YAW_I_VERTICAL, YAW_D_VERTICAL, YAW_I_MAX_VERTICAL
};

class Flight_Controller
{
public:
    Flight_Controller();
    void begin();
    void run();

    struct Input
    {
        uint16_t throttle, roll, pitch, yaw, gear, aux;
    };

    struct Output
    {
        uint16_t right_motor, right_tilt, right_alr;
        uint16_t left_motor, left_tilt, left_alr;
        uint16_t elevator;
    };

    void calculate_outputs(Input& input, Output& output);

    float get_target_roll();
    float get_target_pitch();
    float get_target_yaw();

    float get_roll_error();
    float get_pitch_error();
    float get_yaw_error();

    float get_roll_angle();
    float get_pitch_angle();
    float get_yaw_angle();

    Control_Mode get_control_mode();
    Flight_Mode get_flight_mode();
    int16_t get_transition_state();

private:
    void determine_flight_mode(const Input& input);
    void determine_control_mode(const Input& input);
    void calculate_pids(Input& input, Input& output);
    void calculate_targets(Input& input);
    void map_outputs(Input& input, Output& output);

    float interpolate(float val, float min_from, float max_from, float min_to, float max_to);
    float constrain_angle(float val);
    float constrain_target(float val, float min_val, float max_val);

    Imu imu{ LED_BUILTIN };

    Flight_Mode flight_mode = Flight_Mode::SLOW;
    Control_Mode control_mode = Control_Mode::MANUAL;

    PIDcontroller roll_pid{ FORWARD_ROLL_GAINS, SLOW_ROLL_GAINS, VERTICAL_ROLL_GAINS, flight_mode };
    PIDcontroller pitch_pid{ FORWARD_PITCH_GAINS, SLOW_PITCH_GAINS, VERTICAL_PITCH_GAINS, flight_mode };
    PIDcontroller yaw_pid{ FORWARD_YAW_GAINS, SLOW_YAW_GAINS, VERTICAL_YAW_GAINS, flight_mode };

    FIR_Filter::Response input_response = {
        //0.2, 0.2, 0.2, 0.2, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0
        0.3, 0.2, 0.2, 0.2, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0
        //0.3, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        //0.5, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        //1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    };

    FIR_Filter input_roll_filter{ input_response };
    FIR_Filter input_pitch_filter{ input_response };
    FIR_Filter input_yaw_filter{ input_response };

    float target_roll = 0;
    float target_pitch = 0;
    float target_yaw = 0;

    int16_t transition_state = 0;

    uint16_t right_tilt_last = NEUTRAL_STICK;
    uint16_t left_tilt_last = NEUTRAL_STICK;
};

#endif // FLIGHT_CONTROLLER_H
