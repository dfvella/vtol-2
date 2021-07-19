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

// Returns the time spent so far in the current loop iteration.
int32_t get_time_spent_us(Loop_Timer *lt) {
    return (int32_t) absolute_time_since_us(lt->time);
}

// Blocks calling thread to regulate the frequency of a loop. Returns the time
// since the function was last called in microseconds.
int32_t set_loop_freq(Loop_Timer *lt) {
    if (lt->start) {
        lt->start = false;
        lt->time = get_absolute_time();
        return 0;
    } else {
        int32_t diff_us = get_time_spent_us(lt);
        while (absolute_time_since_us(lt->time) < lt->period_us) {
            tight_loop_contents();
        }
        lt->time = get_absolute_time();
        return diff_us;
    }
}
