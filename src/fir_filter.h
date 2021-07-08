#ifndef __FIR_FILTER_H__
#define __FIR_FILTER_H__

#include "pico/stdlib.h"

#define FIR_BUFFER_SIZE 10

typedef struct {
    const float *response;

    float buffer[FIR_BUFFER_SIZE];
    size_t front;

    uint8_t startup_counter;
} fir_inst_t;

void fir_filter_init(fir_inst_t *filter, const float *response);
void fir_filter_flush(fir_inst_t *filter);
float fir_filter_calculate(fir_inst_t *filter, float input);

#endif // __FIR_FILTER_H__
