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

// Returns the time spent so far in the current loop iteration.
int32_t get_time_spent_us(Loop_Timer *lt);

// Blocks calling thread to regulate the frequency of a loop. Returns the time
// since the function was last called in microseconds
int32_t set_loop_freq(Loop_Timer *lt);

#endif // __TIMING_H__
