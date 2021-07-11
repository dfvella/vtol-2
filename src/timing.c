#include "timing.h"

// returns the time since time t in microseconds
static inline int64_t absolute_time_since_us(absolute_time_t t) {
    return absolute_time_diff_us(t, get_absolute_time());
}

// initializes a Loop_Timer object to run a loop at frequency freq
Loop_Timer create_loop_timer(int freq) {
    Loop_Timer ret = {
        .time = get_absolute_time(),
        .start = true,
        .period_us = 1000000 / freq
    };
    return ret;
}

// Blocks calling thread to regulate the frequency of a loop. If the
// loop frequency is less than freq, the function returns the loop period.
// Otherwise the function returns 0.
// freq units: Hz
int32_t set_loop_freq(Loop_Timer *lt) {
    if (lt->start) {
        lt->start = false;
        lt->time = get_absolute_time();
        return 0;
    } else {
        int64_t diff_us = absolute_time_since_us(lt->time);
        while (absolute_time_since_us(lt->time) < lt->period_us) {
            tight_loop_contents();
        }
        lt->time = get_absolute_time();
        return (int32_t)diff_us;
    }
}
