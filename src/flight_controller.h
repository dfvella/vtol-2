#ifndef __FLIGHT_CONTROLLER_H__
#define __FLIGHT_CONTROLLER_H__

#include <mpu6050.h>

#include "pico/stdlib.h"

#include "pid_controller.h"

// Three position switch thresholds
#define FC_MODE_SWITCH_THRESHOLD_1 -50
#define FC_MODE_SWITCH_THRESHOLD_2 50

#define FC_MIN_ATTITUDE -100
#define FC_MAX_ATTITUDE  100

#define FC_MIN_TSTATE 0
#define FC_MAX_TSTATE 90
#define FC_HORZ_TSTATE FC_MIN_TSTATE
#define FC_VERT_TSTATE FC_MAX_TSTATE

#define FC_MIN_OUTPUT -100
#define FC_MAX_OUTPUT 100

#define FC_DEAD_STICK 5

#define FC_ANGLE_MAX_PITCH_TARGET   60  // units: degrees
#define FC_ANGLE_MAX_ROLL_TARGET    60  // units: degrees
#define FC_ANGLE_MAX_YAW_RATE       2.5 // units: degrees per 20 ms
#define FC_ANGLE_MAX_YAW_ERROR      30  // units: degrees

#define FC_RATE_MAX_ROLL_RATE       1.5 // units: degrees per 20 ms
#define FC_RATE_MAX_PITCH_RATE      1.0 // units: degrees per 20 ms
#define FC_RATE_MAX_YAW_RATE        2.5 // units: degrees per 20 ms
#define FC_RATE_MAX_ROLL_ERROR      30  // units: degrees
#define FC_RATE_MAX_PITCH_ERROR     30  // units: degrees
#define FC_RATE_MAX_YAW_ERROR       30  // units: degrees

#define FC_INVERT_ROLL 1 // set as 1 to invert roll input from imu
#define FC_INVERT_PITCH 0 // set as 1 to invert pitch input from imu
#define FC_INVERT_YAW 0 // set as 1 to invert yaw input from imu

// ********** Horizontal Flight PID Gains ********** //
#define FC_HORZ_ROLL_P      3.0
#define FC_HORZ_ROLL_I      0.0
#define FC_HORZ_ROLL_D      0.0
#define FC_HORZ_ROLL_I_MAX  1.0

#define FC_HORZ_PITCH_P     3.0
#define FC_HORZ_PITCH_I     0.0
#define FC_HORZ_PITCH_D     0.0
#define FC_HORZ_PITCH_I_MAX 1.0

#define FC_HORZ_YAW_P       3.0
#define FC_HORZ_YAW_I       0.0
#define FC_HORZ_YAW_D       0.0
#define FC_HORZ_YAW_I_MAX   1.0

// ********** Vertical Flight PID Gains ********** //
#define FC_VERT_ROLL_P      3.0
#define FC_VERT_ROLL_I      0.0
#define FC_VERT_ROLL_D      0.0
#define FC_VERT_ROLL_I_MAX  1.0

#define FC_VERT_PITCH_P     3.0
#define FC_VERT_PITCH_I     0.0
#define FC_VERT_PITCH_D     0.0
#define FC_VERT_PITCH_I_MAX 1.0

#define FC_VERT_YAW_P       3.0
#define FC_VERT_YAW_I       0.0
#define FC_VERT_YAW_D       0.0
#define FC_VERT_YAW_I_MAX   1.0

// ********** Control Map/Mix Gains ********** //
#define FC_HORZ_YAW_DIFF    0.2
#define FC_VERT_ROLL_DIFF   0.2

typedef enum {
    FC_RX_FAILED = 1,
    FC_IMU_FAILED = 2,
    FC_BMP_FAILED = 4,
    FC_WD_REBOOT = 8,
    FC_OVERRUN = 16
} Fc_Flags;

typedef enum {
    FC_CTRL_MANUAL = 0,
    FC_CTRL_RATE = 1,
    FC_CTRL_ANGLE = 2
} Fc_Ctrl_Mode;

typedef enum {
    FC_FMODE_DISABLED = 0,
    FC_FMODE_VERTICAL = 1,
    FC_FMODE_HORIZONTAL = 2
} Fc_Flight_Mode;

typedef struct {
    float thro;
    float elev;
    float rudd;
    float aile;
    float gear;
    float aux1;

    quaternion_t orientation;
    float alt;
} Fc_Input;

typedef struct {
    float thro;
    float roll;
    float pitch;
    float yaw;
} Fc_Pid_Output;

typedef struct {
    float right_elevon;
    float left_elevon;
    float right_motor;
    float left_motor;
    float gear;
} Fc_Output;

typedef struct {
    Fc_Ctrl_Mode ctrl_mode;
    Fc_Flight_Mode flight_mode;

    float target_roll;
    float target_pitch;
    float target_yaw;

    // number between 0 and 100 inclusive
    // 0 - in horizontal flight mode
    // 90 - in vertical flight mode
    // other values indicate transition between flight modes
    // corresponds to pitch offset from horizontal
    int8_t tstate;

    pid_inst_t pid_roll;
    pid_inst_t pid_pitch;
    pid_inst_t pid_yaw;

    float pid_output_roll;
    float pid_output_pitch;
    float pid_output_yaw;

    // save most recent input/outputs for logging
    Fc_Input input;
    Fc_Flags flags;
    Fc_Output output;
    float roll;
    float pitch;
    float yaw;
} Fc_State;

Fc_Output fc_calc(const Fc_Input* input, Fc_Flags flags);
const Fc_State *fc_get_state(void);

#endif // __FLIGHT_CONTROLLER_H__
