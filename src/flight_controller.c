#include "flight_controller.h"

static Fc_State fc;

static inline float absf(float val) {
    if (val < 0) {
        return val * -1;
    } else {
        return val;
    }
}

static inline float interpolate(
    float val,
    float min_from,
    float max_from,
    float min_to,
    float max_to
) {
    return (((val - min_from) / (max_from - min_from)) * (max_to - min_to)) + min_to;
}

static inline int8_t constrain8(int8_t val, int8_t min, int8_t max) {
    if (val < min) {
        return min;
    } else if (val > max) {
        return max;
    } else {
        return val;
    }
}

static inline float constrainf(float val, float min, float max) {
    if (val < min) {
        return min;
    } else if (val > max) {
        return max;
    } else {
        return val;
    }
}

static inline float constrain_output(float val) {
    return constrainf(val, FC_MIN_OUTPUT, FC_MAX_OUTPUT);
}

static inline float constrain_angle(float angle) {
    if (angle > 180) {
        return angle - 360;
    } else if (angle < -180) {
        return angle + 360;
    } else {
        return angle;
    }
}

static float optimize_target(float target, float current, float max_error) {
    float bound_a = constrain_angle(current - max_error);
    float bound_b = constrain_angle(current + max_error);

    float cost_a = absf(constrain_angle(target - bound_a));
    float cost_b = absf(constrain_angle(target - bound_b));

    if (cost_a < 2 * max_error && cost_b < 2 * max_error) {
        return target;
    } else {
        if (cost_a < cost_b) {
            return bound_a;
        } else {
            return bound_b;
        }
    }
}

static Fc_Ctrl_Mode get_ctrl_mode(const Fc_Input *input, Fc_Flags flags) {
    Fc_Ctrl_Mode mode = FC_CTRL_ANGLE;

    if (input->aux1 < FC_MODE_SWITCH_THRESHOLD_2) {
        mode = FC_CTRL_RATE;
    }
    
    if (input->aux1 < FC_MODE_SWITCH_THRESHOLD_1) {
        mode = FC_CTRL_MANUAL;
    }

    // ===== check for system failures =====
    // handle receiver failure
    // handle imu failure
    if (flags & FC_RX_FAILED) {
        mode = FC_CTRL_ANGLE;
    }
    if (flags & FC_IMU_FAILED) {
        mode = FC_CTRL_MANUAL;
    }

    return mode;
}

static Fc_Flight_Mode get_flight_mode(const Fc_Input *input, Fc_Flags flags) {
    Fc_Flight_Mode mode = FC_FMODE_DISABLED;

    if (input->gear < FC_MODE_SWITCH_THRESHOLD_2) {
        mode = FC_FMODE_VERTICAL;
    }

    if (input->gear < FC_MODE_SWITCH_THRESHOLD_1) {
        mode = FC_FMODE_HORIZONTAL;
    }

    // ===== check for system failures =====
    // handle receiver and imu failure
    // handle receiver failure
    // handle imu failure
    if ((flags & FC_RX_FAILED) && (flags & FC_IMU_FAILED)) {
        mode = FC_FMODE_DISABLED;
    }
    else if ((flags & FC_RX_FAILED) || (flags & FC_IMU_FAILED)) {
        mode = FC_FMODE_HORIZONTAL;
    }

    return mode;
}

static float get_target_roll(float input, float curr, float target, Fc_Ctrl_Mode mode) {
    static bool start = true;

    if (start) {
        start = false;
        target = curr;
    } else {
        switch (mode) {
        case FC_CTRL_ANGLE:
            target = interpolate(input,
                -FC_MAX_ATTITUDE, FC_MAX_ATTITUDE,
                -FC_ANGLE_MAX_ROLL_TARGET, FC_ANGLE_MAX_ROLL_TARGET
            );
            break;
        case FC_CTRL_RATE:
            target += interpolate(input,
                -FC_MAX_ATTITUDE, FC_MAX_ATTITUDE,
                -FC_RATE_MAX_ROLL_RATE, FC_RATE_MAX_ROLL_RATE
            );
            target = optimize_target(target, curr, FC_RATE_MAX_ROLL_ERROR);
            break;
        case FC_CTRL_MANUAL:
            target = curr;
            break;
        }
    }
    return target;
}

static float get_target_pitch(float input, float curr, float target, Fc_Ctrl_Mode mode) {
    static bool start = true;

    if (start) {
        start = false;
        target = curr;
    } else {
        switch (mode) {
        case FC_CTRL_ANGLE:
            target = interpolate(input,
                -FC_MAX_ATTITUDE, FC_MAX_ATTITUDE,
                -FC_ANGLE_MAX_PITCH_TARGET, FC_ANGLE_MAX_PITCH_TARGET
            );
            break;
        case FC_CTRL_RATE:
            target += interpolate(input,
                -FC_MAX_ATTITUDE, FC_MAX_ATTITUDE,
                -FC_RATE_MAX_PITCH_RATE, FC_RATE_MAX_PITCH_RATE
            );
            target = optimize_target(target, curr, FC_RATE_MAX_PITCH_ERROR);
            break;
        case FC_CTRL_MANUAL:
            target = curr;
            break;
        }
    }
    return target;
}

static float get_target_yaw(float input, float curr, float target, Fc_Ctrl_Mode mode) {
    static bool start = true;

    if (start) {
        start = false;
        target = curr;
    } else {
        switch (mode) {
        case FC_CTRL_MANUAL:
            target = curr;
            break;
        default:
            target += interpolate(input,
                -FC_MAX_ATTITUDE, FC_MAX_ATTITUDE,
                -FC_RATE_MAX_YAW_RATE, FC_RATE_MAX_YAW_RATE
            );
            target = optimize_target(target, curr, FC_RATE_MAX_YAW_ERROR);
            break;
        }
    }
    return target;
}

static int8_t get_transition_state(int8_t state, Fc_Flight_Mode mode) {
    static bool start = true;

    if (start) {
        start = false;

        switch (mode) {
        case FC_FMODE_HORIZONTAL:
            state =  FC_MIN_TSTATE;
            break;
        default:
            state = FC_MAX_TSTATE;
            break;
        }
    } else {
        switch (mode) {
        case FC_FMODE_HORIZONTAL:
            --state;
            break;
        default:
            ++state;
            break;
        }
    }
    return constrain8(state, FC_MIN_TSTATE, FC_MAX_TSTATE);
}

static float get_roll_pid(pid_inst_t *pid, float error, int8_t tstate) {
    static bool start = true;

    float p = interpolate(tstate,
        FC_HORZ_TSTATE, FC_VERT_TSTATE,
        FC_HORZ_ROLL_P, FC_VERT_ROLL_P
    );
    float i = interpolate(tstate,
        FC_HORZ_TSTATE, FC_VERT_TSTATE,
        FC_HORZ_ROLL_I, FC_VERT_ROLL_I
    );
    float d = interpolate(tstate,
        FC_HORZ_TSTATE, FC_VERT_TSTATE,
        FC_HORZ_ROLL_D, FC_VERT_ROLL_D
    );
    float i_max = interpolate(tstate,
        FC_HORZ_TSTATE, FC_VERT_TSTATE,
        FC_HORZ_ROLL_I_MAX, FC_VERT_ROLL_I_MAX
    );

    if (start) {
        start = false;
        pid_init(pid, p, i, d, i_max);
    } else {
        pid_set_gains(pid, p, i, d, i_max);
    }

    return pid_calculate(pid, error);
}

static float get_pitch_pid(pid_inst_t *pid, float error, int8_t tstate) {
    static bool start = true;

    float p = interpolate(tstate,
        FC_HORZ_TSTATE, FC_VERT_TSTATE,
        FC_HORZ_PITCH_P, FC_VERT_PITCH_P
    );
    float i = interpolate(tstate,
        FC_HORZ_TSTATE, FC_VERT_TSTATE,
        FC_HORZ_PITCH_I, FC_VERT_PITCH_I
    );
    float d = interpolate(tstate,
        FC_HORZ_TSTATE, FC_VERT_TSTATE,
        FC_HORZ_PITCH_D, FC_VERT_PITCH_D
    );
    float i_max = interpolate(tstate,
        FC_HORZ_TSTATE, FC_VERT_TSTATE,
        FC_HORZ_PITCH_I_MAX, FC_VERT_PITCH_I_MAX
    );

    if (start) {
        start = false;
        pid_init(pid, p, i, d, i_max);
    } else {
        pid_set_gains(pid, p, i, d, i_max);
    }

    return pid_calculate(pid, error);
}

static float get_yaw_pid(pid_inst_t *pid, float error, int8_t tstate) {
    static bool start = true;

    float p = interpolate(tstate,
        FC_HORZ_TSTATE, FC_VERT_TSTATE,
        FC_HORZ_YAW_P, FC_VERT_YAW_P
    );
    float i = interpolate(tstate,
        FC_HORZ_TSTATE, FC_VERT_TSTATE,
        FC_HORZ_YAW_I, FC_VERT_YAW_I
    );
    float d = interpolate(tstate,
        FC_HORZ_TSTATE, FC_VERT_TSTATE,
        FC_HORZ_YAW_D, FC_VERT_YAW_D
    );
    float i_max = interpolate(tstate,
        FC_HORZ_TSTATE, FC_VERT_TSTATE,
        FC_HORZ_YAW_I_MAX, FC_VERT_YAW_I_MAX
    );

    if (start) {
        start = false;
        pid_init(pid, p, i, d, i_max);
    } else {
        pid_set_gains(pid, p, i, d, i_max);
    }

    return pid_calculate(pid, error);
}

float get_roll(const quaternion_t *q) {
    float roll = quaternion_get_roll(q);

#   if FC_INVERT_ROLL == 1
        roll  *= -1;
#   endif

    return roll;
}

float get_pitch(const quaternion_t *q) {
    float pitch = quaternion_get_pitch(q);

#   if FC_INVERT_PITCH == 1
        pitch *= -1;
#   endif

    return pitch;
}

float get_yaw(const quaternion_t *q) {
    float yaw = quaternion_get_yaw(q);

#   if FC_INVERT_YAW == 1
        yaw *= -1;
#   endif

    return yaw;
}

static inline bool use_horz_ctrls(float tstate) {
    return tstate < FC_MAX_TSTATE / 2;
}

static float map_right_elevon(const Fc_Command *command, float tstate) {
    float output = command->pitch;

    if (use_horz_ctrls(tstate)) {
        output += command->roll;
    } else {
        output -= command->yaw;
    }

    return constrainf(output, FC_MIN_OUTPUT, FC_MAX_OUTPUT);
}

static float map_left_elevon(const Fc_Command *command, float tstate) {
    float output = command->pitch;

    if (use_horz_ctrls(tstate)) {
        output -= command->roll;
    } else {
        output += command->yaw;
    }

    return constrainf(output, FC_MIN_OUTPUT, FC_MAX_OUTPUT);
}

static float map_right_motor(const Fc_Command *command, float tstate) {
    float output = command->thro;

    if (use_horz_ctrls(tstate)) {
        output -= command->yaw * FC_HORZ_YAW_DIFF;
    } else {
        output -= command->roll * FC_VERT_ROLL_DIFF;
    }

    return constrainf(output, FC_MIN_OUTPUT, FC_MAX_OUTPUT);
}

static float map_left_motor(const Fc_Command *command, float tstate) {
    float output = command->thro;

    if (use_horz_ctrls(tstate)) {
        output += command->yaw * FC_HORZ_YAW_DIFF;
    } else {
        output += command->roll * FC_VERT_ROLL_DIFF;
    }

    return constrainf(output, FC_MIN_OUTPUT, FC_MAX_OUTPUT);
}

static float map_gear(float tstate) {
    float output;

    if (tstate < FC_MAX_TSTATE / 2) {
        output = FC_MIN_OUTPUT;
    } else {
        output = interpolate(tstate,
            FC_MAX_TSTATE / 2, FC_MAX_TSTATE,
            FC_MIN_OUTPUT, FC_MAX_OUTPUT
        );
    }

    return constrainf(output, FC_MIN_OUTPUT, FC_MAX_OUTPUT);
}

static Fc_Output get_output(
    const Fc_Input *input,
    const Fc_Pid_Output *pid_out,
    Fc_Ctrl_Mode ctrl_mode,
    float tstate
) {
    Fc_Output output;
    Fc_Command command;

    command.thro = input->thro;

    switch (ctrl_mode) {
    case FC_CTRL_MANUAL:
        command.roll = input->aile;
        command.pitch = input->elev;
        command.yaw = input->rudd;
        break;
    default:
        command.roll = pid_out->roll;
        command.pitch = pid_out->pitch;
        command.yaw = pid_out->yaw;
        break;
    }

    output.right_elevon = map_right_elevon(&command, tstate);
    output.left_elevon = map_left_elevon(&command, tstate);
    output.right_motor = map_right_motor(&command, tstate);
    output.left_motor = map_left_motor(&command, tstate);
    output.gear = map_gear(tstate);

    // only enable throttle mixing if there is some input throttle
    if (input->thro < FC_MIN_OUTPUT + FC_DEAD_STICK) {
        output.right_motor = FC_MIN_OUTPUT;
        output.left_motor = FC_MIN_OUTPUT;
    }

    return output;
}

const Fc_Output *fc_calc(const Fc_Input *input, Fc_Flags flags) {
    fc.input = *input;
    fc.flags = flags;

    fc.ctrl_mode = get_ctrl_mode(&fc.input, fc.flags);
    fc.flight_mode = get_flight_mode(&fc.input, fc.flags);

    if (fc.flight_mode == FC_FMODE_DISABLED || fc.flags & FC_RX_FAILED) {
        fc.input.thro = FC_MIN_INPUT;
        fc.input.aile = FC_CEN_INPUT;
        fc.input.elev = FC_CEN_INPUT;
        fc.input.rudd = FC_CEN_INPUT;
    }

    fc.tstate = get_transition_state(fc.tstate, fc.flight_mode);

    // compensate pitch based on transition state.
    quaternion_t q = quaternion_rotate_pitch(&fc.input.orientation, fc.tstate * -1);

    fc.roll = get_roll(&q);
    fc.pitch = get_pitch(&q);
    fc.yaw = get_yaw(&q);

    fc.target_roll = get_target_roll(fc.input.aile, fc.roll, fc.target_roll, fc.ctrl_mode);
    fc.target_pitch = get_target_pitch(fc.input.elev, fc.pitch, fc.target_pitch, fc.ctrl_mode);
    fc.target_yaw = get_target_yaw(fc.input.rudd, fc.yaw, fc.target_yaw, fc.ctrl_mode);

    float error_roll = fc.target_roll - fc.roll;
    float error_pitch = fc.target_pitch - fc.pitch;
    float error_yaw = fc.target_yaw - fc.yaw;

    fc.pid_out.thro = fc.input.thro;
    fc.pid_out.roll = get_roll_pid(&fc.pid_roll, error_roll, fc.tstate);
    fc.pid_out.pitch = get_pitch_pid(&fc.pid_pitch, error_pitch, fc.tstate);
    fc.pid_out.yaw = get_yaw_pid(&fc.pid_yaw, error_yaw, fc.tstate);

    fc.output = get_output(&fc.input, &fc.pid_out, fc.ctrl_mode, fc.tstate);

    // save original input for logging
    fc.input = *input;

    return &fc.output;
}

const Fc_State *fc_get_state(void) {
    return &fc;
}
