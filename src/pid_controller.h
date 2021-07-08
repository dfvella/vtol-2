#ifndef __PID_CONTROLLER_H__
#define __PID_CONTROLLER_H__

#include "pico/stdlib.h"
#include "pico/time.h"

#include "fir_filter.h"

#define PID_MAX_OUTPUT 100

typedef struct {
    float p;
    float i;
    float d;
    float i_max;

    float i_output;
    float prev_error;

    uint8_t start;
    absolute_time_t time;

    float d_response[FIR_BUFFER_SIZE];

    fir_inst_t d_filter;
} pid_inst_t;

void pid_init(pid_inst_t *pid, float p, float i, float d, float i_max);
void pid_set_gains(pid_inst_t *pid, float p, float i, float d, float i_max);
float pid_calculate(pid_inst_t *pid, float error);

#endif // __PID_CONTROLLER_H__
