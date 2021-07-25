#include <mpu6050.h>
#include <ar610.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "constants.h"
#include "flight_controller.h"
#include "logging.h"
#include "pwm.h"
#include "reboot.h"

#include <stdio.h>

typedef enum {
    RUN_BMP_REQ = 0,
    RUN_AR_GET = 1,
    RUN_BMP_GET = 2,
    RUN_FC_CALC = 3,
    RUN_SERV_SET = 4
} Loop_State;

static int hardware_init(i2c_inst_t* i2c, mpu6050_inst_t* mpu, ar610_inst_t* ar);

static void loop(mpu6050_inst_t *mpu, ar610_inst_t *ar);
static void run_bmp_req();
static Fc_Input run_ar_get(mpu6050_inst_t *mpu, ar610_inst_t *ar, Fc_Flags *flags);
static void run_bmp_get();
static Fc_Output run_fc_calc(const Fc_Input *input, Fc_Flags *flags);
static void run_serv_set(const Fc_Output *output);

int main() {
    // ESC pins must be configured immediately.
    // Otherwise the ESC will enter a failure state.
    pwm_esc_patch();

    mpu6050_inst_t mpu;
    ar610_inst_t ar610;
    i2c_inst_t *i2c = &i2c0_inst;

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
            printf("info: starting flight controller\n");
            int err = hardware_init(i2c, &mpu, &ar610);
            if (err) {
                reboot();
            }
            for (;;) {
                loop(&mpu, &ar610);
            }
        } else {
            printf("error: unrecognized command\n");
        }

        start = false;
    }
}

int hardware_init(i2c_inst_t* i2c, mpu6050_inst_t* mpu, ar610_inst_t* ar) {
    // Initialize status led
    gpio_init(STATUS_LED_PIN);
    gpio_set_dir(STATUS_LED_PIN, GPIO_OUT);

    // Initialize flash logging
    printf("info: initializing flash logging ...\n");
    init_logging();

    // Initialize I2C bus
    i2c_init(i2c, I2C_BAUD_RATE_HZ);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // Initialize radio receiver
    printf("info: initializing radio receiver ...\n");
    ar610_init(ar,
        AR610_THRO_PIN,
        AR610_AILE_PIN,
        AR610_ELEV_PIN,
        AR610_RUDD_PIN,
        AR610_GEAR_PIN,
        AR610_AUX1_PIN
    );

    // Initialize pwm outputs
    printf("info: initializing pwm outputs ...\n");
    pwm_init_all_outputs();

    // Initialize IMU
    printf("info: initializing imu ...\n");
    uint8_t imu_error = mpu6050_init(mpu, i2c, STATUS_LED_PIN);
    if (imu_error) {
        printf("error: the imu was not found on the bus\n");
        return 1;
    }

    return 0;
}

void run_bmp_req() {
    return;
}

Fc_Input run_ar_get(mpu6050_inst_t *mpu, ar610_inst_t *ar, Fc_Flags *flags) {
    Fc_Input input;

    ar610_update_state(ar);

    if (!ar610_is_connected(ar)) {
        *flags |= FC_RX_FAILED;
    }

    input.thro = ar610_get_thro(ar);
    input.elev = ar610_get_elev(ar);
    input.aile = ar610_get_aile(ar);
    input.rudd = ar610_get_rudd(ar);
    input.gear = ar610_get_gear(ar);
    input.aux1 = ar610_get_aux1(ar);

    input.orientation = mpu6050_get_quaternion(mpu);

    return input;
}

void run_bmp_get() {
    return;
}

Fc_Output run_fc_calc(const Fc_Input *input, Fc_Flags *flags) {
    const Fc_Output *output;
    output = fc_calc(input, *flags);
    *flags = 0;
    return *output;
}

void run_serv_set(const Fc_Output *output) {
    const Fc_State *state = fc_get_state();
    if (state->waiting || (state->flight_mode == FC_FMODE_DISABLED)) {
        pwm_disable_all_outputs();
    } else {
        pwm_set_right_elevon(output->right_elevon);
        pwm_set_left_elevon(output->left_elevon);
        pwm_set_right_motor(output->right_motor);
        pwm_set_left_motor(output->left_motor);
    }
    do_logging();
}

void loop(mpu6050_inst_t *mpu, ar610_inst_t *ar) {
    static Loop_State loop_state = RUN_BMP_GET;

    static Fc_Flags fc_flags = 0;
    static Fc_Input fc_input;
    static Fc_Output fc_output;

    static absolute_time_t time = { 0 };

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

    int imu_error = mpu6050_update_state(mpu);
    if (imu_error) {
        fc_flags |= FC_IMU_FAILED;
    }

    if (fc_get_state()->flags) {
        gpio_put(STATUS_LED_PIN, true);
    } else {
        gpio_put(STATUS_LED_PIN, false);
    }

    switch (loop_state) {
    case RUN_BMP_REQ:
        run_bmp_req();
        break;
    case RUN_AR_GET:
        fc_input = run_ar_get(mpu, ar, &fc_flags);
        break;
    case RUN_BMP_GET:
        run_bmp_get();
        break;
    case RUN_FC_CALC:
        fc_output = run_fc_calc(&fc_input, &fc_flags);
        break;
    case RUN_SERV_SET:
        run_serv_set(&fc_output);
        break;
    };

    loop_state = (loop_state + 1) % 5;
}
