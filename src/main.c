#include <mpu6050.h>
#include <ar610.h>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/bootrom.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"

#include "constants.h"
#include "timing.h"
#include "flight_controller.h"
#include "pwm.h"

#include <stdio.h>

#include "logging.h"

#define I2C_INST i2c0_inst

#define STDIO_WAIT 2 // units: seconds
#define USB_WAIT 3 // units: seconds

#define LOOP_FREQ 250 // units: Hertz
#define LOOP_PERIOD_US 1000000 / LOOP_FREQ // unit: microseconds

#define COMMAND_DUMP_LOGS 'd'
#define COMMAND_REBOOT 'r'
#define COMMAND_BOOTSEL 'b'

#define ENABLE_ACTUATORS
//#define PERF_CHECK

static void reboot(void) {
    watchdog_enable(1, 0);
}

static void bootsel(void) {
    reset_usb_boot(0, 0);
}

static int init_all(i2c_inst_t* i2c, mpu6050_inst_t* mpu, ar610_inst_t* ar) {
    // Initialize status led
    gpio_init(STATUS_LED_PIN);
    gpio_set_dir(STATUS_LED_PIN, GPIO_OUT);

    // Initialize I2C bus
    i2c_init(i2c, I2C_BAUD_RATE_HZ);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // Initialize flash logging
    PRINTF_DEBUG("info: initializing flash logging ...\n");
    init_logging();

    // Initialize radio receiver
    PRINTF_DEBUG("info: initializing radio receiver ...\n");
    ar610_init(ar,
        AR610_THRO_PIN,
        AR610_AILE_PIN,
        AR610_ELEV_PIN,
        AR610_RUDD_PIN,
        AR610_GEAR_PIN,
        AR610_AUX1_PIN
    );

    // Initialize pwm outputs
    PRINTF_DEBUG("info: initializing pwm outputs ...\n");
    pwm_init_all_outputs();

    // Initialize IMU
    PRINTF_DEBUG("info: initializing imu ...\n");
    uint8_t imu_error = mpu6050_init(mpu, i2c, STATUS_LED_PIN);
    if (imu_error) {
        PRINTF_DEBUG("error: the imu was not found on the bus\n");
        return 1;
    }

    return 0;
}

void run(mpu6050_inst_t *mpu, ar610_inst_t *ar) {
    enum {
        RUN_BMP_REQ = 0,
        RUN_AR_GET = 1,
        RUN_BMP_GET = 2,
        RUN_FC_CALC = 3,
        RUN_SERV_SET = 4
    } run_state;

    static Fc_Flags fc_flags = 0;
    static Fc_Input fc_input;
    static const Fc_Output *fc_output;

    absolute_time_t var = get_absolute_time();

    Loop_Timer loop_timer = create_loop_timer(LOOP_FREQ);
    for (;;) {
        int imu_error = mpu6050_update_state(mpu);
        if (imu_error) {
            fc_flags |= FC_IMU_FAILED;
        }

        switch (run_state) {
        case RUN_BMP_REQ:
            break;
        case RUN_AR_GET:
            ar610_update_state(ar);

            if (!ar610_is_connected(ar)) {
                fc_flags |= FC_RX_FAILED;
            }

            fc_input.thro = ar610_get_thro(ar);
            fc_input.elev = ar610_get_elev(ar);
            fc_input.aile = ar610_get_aile(ar);
            fc_input.rudd = ar610_get_rudd(ar);
            fc_input.gear = ar610_get_gear(ar);
            fc_input.aux1 = ar610_get_aux1(ar);

            fc_input.orientation = mpu6050_get_quaternion(mpu);
            break;
        case RUN_BMP_GET:
            break;
        case RUN_FC_CALC:
            fc_output = fc_calc(&fc_input, fc_flags);
            fc_flags = 0;
            break;
        case RUN_SERV_SET:
#           ifdef ENABLE_ACTUATORS
                pwm_set_right_elevon(fc_output->right_elevon);
                pwm_set_left_elevon(fc_output->left_elevon);
                pwm_set_right_motor(fc_output->right_motor);
                pwm_set_left_motor(fc_output->left_motor);
#           endif // ENABLE_ACTUATORS
#           ifndef PERF_CHECK
                do_logging();
#           endif // PERF_CHECK
            break;
        };

        run_state = (run_state + 1) % 5;

#       ifdef DEBUG
            int32_t timeout_us = LOOP_PERIOD_US - get_time_spent_us(&loop_timer) - 500;
            if (timeout_us > 0) {
                char ch = getchar_timeout_us(timeout_us);
                if (ch == COMMAND_REBOOT) {
                    reboot();
                } else if (ch == COMMAND_BOOTSEL) {
                    bootsel();
                }
            }
#       endif // DEBUG

        int32_t loop_period_us = set_loop_freq(&loop_timer);
#       ifdef PERF_CHECK
            printf("info: state %d period %d\n", run_state, loop_period_us);
#       endif // PERF_CHECK
        if (loop_period_us > LOOP_PERIOD_US) {
            gpio_put(STATUS_LED_PIN, true);
            fc_flags |= FC_OVERRUN;
        } else {
            gpio_put(STATUS_LED_PIN, false);
        }
    }
}

int main(void) {
    // ESC pins must be configured immediately.
    // Otherwise the ESC will enter a failure state.
    gpio_init(RIGHT_MOTOR_PIN);
    gpio_init(LEFT_MOTOR_PIN);
    gpio_set_dir(RIGHT_MOTOR_PIN, GPIO_OUT);
    gpio_set_dir(LEFT_MOTOR_PIN, GPIO_OUT);

    mpu6050_inst_t mpu;
    ar610_inst_t ar610;
    i2c_inst_t *i2c = &I2C_INST;

    bool start = true;

    stdio_init_all();
    sleep_ms(STDIO_WAIT * 1000);

    for (;;) {
        int ch = getchar_timeout_us(USB_WAIT * 1000000);

        if (ch == COMMAND_DUMP_LOGS) {
            dump_logs();
        } else if (ch == COMMAND_REBOOT) {
            reboot();
        } else if (ch == COMMAND_BOOTSEL) {
            bootsel();
        } else if (start && ch == PICO_ERROR_TIMEOUT) {
            PRINTF_DEBUG("info: starting flight controller\n");

            // initialize hardware
            int init_error = init_all(i2c, &mpu, &ar610);
            if (init_error) {
                reboot();
            }

            // run the flight_controller
            run(&mpu, &ar610);
        } else {
            PRINTF_DEBUG("error: unrecognized command\n");
        }

        start = false;
    }
}
