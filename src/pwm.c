#include "pwm.h"

static uint slice_elevon;
static uint slice_motor;
static uint16_t wrap;

static inline uint16_t pwm_pulse_width_to_level(uint16_t pulse_width) {
    return (pulse_width / 20000.0f) * wrap;
}

void pwm_init_all_outputs() {
    gpio_set_function(RIGHT_ELEVON_PIN, GPIO_FUNC_PWM);
    gpio_set_function(LEFT_ELEVON_PIN, GPIO_FUNC_PWM);
    gpio_set_function(RIGHT_MOTOR_PIN, GPIO_FUNC_PWM);
    gpio_set_function(LEFT_MOTOR_PIN, GPIO_FUNC_PWM);

    wrap = (clock_get_hz(clk_sys) / PWM_CLOCK_DIVIDER) / PWM_FREQUENCY;

    slice_elevon = pwm_gpio_to_slice_num(RIGHT_ELEVON_PIN);
    pwm_set_clkdiv(slice_elevon, PWM_CLOCK_DIVIDER);
    pwm_set_wrap(slice_elevon, wrap);
    pwm_set_chan_level(slice_elevon, PWM_CHAN_A, 0);
    pwm_set_chan_level(slice_elevon, PWM_CHAN_B, 0);
    pwm_set_enabled(slice_elevon, true);

    slice_motor = pwm_gpio_to_slice_num(RIGHT_MOTOR_PIN);
    pwm_set_clkdiv(slice_motor, PWM_CLOCK_DIVIDER);
    pwm_set_wrap(slice_motor, wrap);
    pwm_set_chan_level(slice_motor, PWM_CHAN_A, 0);
    pwm_set_chan_level(slice_motor, PWM_CHAN_B, 0);
    pwm_set_enabled(slice_motor, true);
}

void pwm_set_right_elevon(uint16_t pulse_width) {
    uint16_t level = pwm_pulse_width_to_level(pulse_width);
    pwm_set_chan_level(slice_elevon, PWM_CHAN_A, level);
}

void pwm_set_left_elevon(uint16_t pulse_width);

void pwm_set_right_motor(uint16_t pulse_width);
void pwm_set_left_motor(uint16_t pulse_width);
