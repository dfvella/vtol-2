#ifndef __TIMING_H__
#define __TIMING_H__

#include "pico/stdlib.h"
#include "pico/time.h"

typedef struct {
    absolute_time_t time;
    bool start;
    int32_t period_us;
} Loop_Timer;

// initializes a Loop_Timer object to run a loop at frequency freq
Loop_Timer create_loop_timer(int freq);

// Blocks calling thread to regulate the frequency of a loop. If the
// loop frequency is less than freq, the function returns the loop period.
// Otherwise the function returns 0.
// freq units: Hz
int32_t set_loop_freq(Loop_Timer *lt);

#endif // __TIMING_H__
