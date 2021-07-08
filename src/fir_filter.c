#include "fir_filter.h"

void fir_filter_init(fir_inst_t *filter, const float *response) {
    filter->response = response;
    filter->front = 0;
    filter->startup_counter = 0;
}

void fir_filter_flush(fir_inst_t *filter) {
    filter->startup_counter = 0;
}

float fir_filter_calculate(fir_inst_t *filter, float input) {
    filter->front = (filter->front + 1) % FIR_BUFFER_SIZE;

    filter->buffer[filter->front] = input;

    if (filter->startup_counter < FIR_BUFFER_SIZE) {
        ++filter->startup_counter;
        return input;
    }

    float result = 0;

    for (size_t i = 0; i < FIR_BUFFER_SIZE; ++i) {
        int buffer_index = filter->front - i;
        if (buffer_index < 0) {
            buffer_index += FIR_BUFFER_SIZE;
        }
        result += filter->buffer[buffer_index] * filter->response[i];
    }
    return result;
}
