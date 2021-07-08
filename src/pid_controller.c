#include "pid_controller.h"

static float constrain(float val, float min, float max) {
    if (val > max) {
        return max;
    } else if (val < min) {
        return min;
    } else {
        return val;
    }
}

void pid_init(pid_inst_t *pid, float p, float i, float d, float i_max) {
    pid->p = p;
    pid->i = i;
    pid->d = d;
    pid->i_max = i_max;

    pid->i_output = 0;
    pid->prev_error = 0;

    for (size_t i = 0; i < FIR_BUFFER_SIZE; ++i) {
        pid->d_response[i] = 0;
    }
    pid->d_response[0] = 0.4;
    pid->d_response[1] = 0.3;
    pid->d_response[2] = 0.2;
    pid->d_response[3] = 0.1;

    fir_filter_init(&pid->d_filter, pid->d_response);

    pid->start = 1;
}

void pid_set_gains(pid_inst_t *pid, float p, float i, float d, float i_max) {
    pid->p = p;
    pid->i = i;
    pid->d = d;
    pid->i_max = i_max;
}

float pid_calculate(pid_inst_t *pid, float error) {
    if (pid->start) {
        pid->start = 0;
        pid->time = get_absolute_time();
        return 0;
    } else {
        int64_t t_delta_us = absolute_time_diff_us(pid->time, get_absolute_time());
        pid->time = get_absolute_time();

        float t_delta = (float)t_delta_us / 1000000.0f;

        float output = pid->p * error;

        pid->i_output += t_delta * error;
        pid->i_output = constrain(pid->i_output, (-1 * pid->i_max) / pid->i, pid->i_max / pid->i);
        output += constrain(pid->i * pid->i_output, -1 * pid->i_max, pid->i_max);

        float d_error = fir_filter_calculate(&pid->d_filter,
            (error - pid->prev_error) / t_delta
        );

        output += pid->d * d_error;
        pid->prev_error = error;

        return constrain(output, -PID_MAX_OUTPUT, PID_MAX_OUTPUT);
    }
}
