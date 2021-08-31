// exectuable for flight controller regression test

#include <stdio.h>

#include "pico/stdlib.h"

#include "constants.h"
#include "flight_controller.h"
#include "logging.h"
#include "reboot.h"

#include "sim_input.h"

typedef enum {
    RUN_BMP_REQ = 0,
    RUN_AR_GET = 1,
    RUN_BMP_GET = 2,
    RUN_FC_CALC = 3,
    RUN_SERV_SET = 4
} Loop_State;

void loop() {
    static Loop_State loop_state = RUN_BMP_GET;

    static Fc_Flags fc_flags = 0;
    static Fc_Input fc_input;
    static Fc_Output fc_output;

    static absolute_time_t time = { 0 };

    static Fc_Input *sim_input_ptr = sim_input;

    int32_t diff_us;
    if (to_us_since_boot(time) == 0) { // takes if first call
        diff_us = LOOP_PERIOD_US - (2 * USB_TIMEOUT_PADDING_US);
    } else {
        diff_us = absolute_time_diff_us(time, get_absolute_time());
    }

    int32_t timeout_us = LOOP_PERIOD_US - diff_us - USB_TIMEOUT_PADDING_US;
    if (timeout_us > 0) {
        char ch = getchar_timeout_us(timeout_us);
        if (ch == COMMAND_REBOOT) {
            reboot();
        } else if (ch == COMMAND_BOOTSEL) {
            bootsel();
        }
    }

    diff_us = absolute_time_diff_us(time, get_absolute_time());
    if (diff_us < LOOP_PERIOD_US) {
        while (diff_us < LOOP_PERIOD_US) {
            diff_us = absolute_time_diff_us(time, get_absolute_time());
        }
    } else {
        fc_flags |= FC_OVERRUN;
    }
    time = get_absolute_time();

    switch (loop_state) {
    case RUN_BMP_REQ:
        break;
    case RUN_AR_GET:
        fc_input = *sim_input_ptr;
        ++sim_input_ptr;
        break;
    case RUN_BMP_GET:
        break;
    case RUN_FC_CALC:
        fc_output = *fc_calc(&fc_input, fc_flags);
        fc_flags = 0;
        break;
    case RUN_SERV_SET:
        do_logging();
        break;
    };

    loop_state = (loop_state + 1) % 5;
}

int main() {
    stdio_init_all();
    sleep_ms(3000);

    for (size_t i = 0; i < SIM_INPUT_SIZE * 5; ++i) {
        loop();
    }

    bootsel();
}
