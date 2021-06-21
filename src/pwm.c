#include "pwm.h"

static uint32_t counter_wrap;

// convert PWM pulse width in microseconds to 32 bit integer to compare
// with counter
static inline uint32_t pwm_pulse_width_to_level(uint16_t pulse_width) {
    return (pulse_width / PWM_PERIOD_US) * counter_wrap;
}

// sets the pwm counter wrap
static void pwm_set_wrap(PIO pio, uint sm, uint32_t period) {
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_put_blocking(pio, sm, period);
    pio_sm_exec(pio, sm, pio_encode_pull(false, false));
    pio_sm_exec(pio, sm, pio_encode_out(pio_isr, 32));
    pio_sm_set_enabled(pio, sm, true);
}

void pwm_init_all_outputs() {
    counter_wrap = clock_get_hz(clk_sys) / (PWM_CYCLES * PWM_FREQUENCY);

    uint pio0_offset = pio_add_program(pio0, &pwm_program);
    uint pio1_offset = pio_add_program(pio1, &pwm_program);

    pwm_program_init(pio0, PWM_SM_RIGHT_ELEVON, pio0_offset, RIGHT_ELEVON_PIN);
    pwm_program_init(pio0, PWM_SM_LEFT_ELEVON, pio0_offset, LEFT_ELEVON_PIN);
    pwm_program_init(pio0, PWM_SM_RIGHT_MOTOR, pio0_offset, RIGHT_MOTOR_PIN);
    pwm_program_init(pio0, PWM_SM_LEFT_MOTOR, pio0_offset, LEFT_MOTOR_PIN);

    pwm_set_wrap(pio0, PWM_SM_RIGHT_ELEVON, counter_wrap);
    pwm_set_wrap(pio0, PWM_SM_LEFT_ELEVON, counter_wrap);
    pwm_set_wrap(pio0, PWM_SM_RIGHT_MOTOR, counter_wrap);
    pwm_set_wrap(pio0, PWM_SM_LEFT_MOTOR, counter_wrap);
}

void pwm_set_right_elevon(uint16_t pulse_width) {
    uint32_t level = pwm_pulse_width_to_level(pulse_width);
    pio_sm_put(pio0, PWM_SM_RIGHT_ELEVON, level);
}

void pwm_set_left_elevon(uint16_t pulse_width) {
    uint32_t level = pwm_pulse_width_to_level(pulse_width);
    pio_sm_put(pio0, PWM_SM_LEFT_ELEVON, level);
}

void pwm_set_right_motor(uint16_t pulse_width) {
    uint32_t level = pwm_pulse_width_to_level(pulse_width);
    pio_sm_put(pio0, PWM_SM_RIGHT_MOTOR, level);
}

void pwm_set_left_motor(uint16_t pulse_width) {
    uint32_t level = pwm_pulse_width_to_level(pulse_width);
    pio_sm_put(pio0, PWM_SM_LEFT_MOTOR, level);
}

//void pwm_init_all_outputs() {
//    gpio_set_function(RIGHT_ELEVON_PIN, GPIO_FUNC_PWM);
//    gpio_set_function(LEFT_ELEVON_PIN, GPIO_FUNC_PWM);
//    gpio_set_function(RIGHT_MOTOR_PIN, GPIO_FUNC_PWM);
//    gpio_set_function(LEFT_MOTOR_PIN, GPIO_FUNC_PWM);
//
//    wrap = (clock_get_hz(clk_sys) / PWM_CLOCK_DIVIDER) / PWM_FREQUENCY;
//
//    slice_elevon = pwm_gpio_to_slice_num(RIGHT_ELEVON_PIN);
//    pwm_set_clkdiv(slice_elevon, PWM_CLOCK_DIVIDER);
//    pwm_set_wrap(slice_elevon, wrap);
//    pwm_set_chan_level(slice_elevon, PWM_CHAN_A, 0);
//    pwm_set_chan_level(slice_elevon, PWM_CHAN_B, 0);
//    pwm_set_enabled(slice_elevon, true);
//
//    slice_motor = pwm_gpio_to_slice_num(RIGHT_MOTOR_PIN);
//    pwm_set_clkdiv(slice_motor, PWM_CLOCK_DIVIDER);
//    pwm_set_wrap(slice_motor, wrap);
//    pwm_set_chan_level(slice_motor, PWM_CHAN_A, 0);
//    pwm_set_chan_level(slice_motor, PWM_CHAN_B, 0);
//    pwm_set_enabled(slice_motor, true);
//
//    right_aile_pulse_prev = PWM_MID_PULSEWIDTH;
//}
//
//void pwm_set_right_elevon(uint16_t pulse_width) {
//    uint16_t level = pwm_pulse_width_to_level(pulse_width);
//    pwm_set_chan_level(slice_elevon, PWM_CHAN_A, level);
//}
//
//void pwm_set_left_elevon(uint16_t pulse_width) {
//    uint16_t level = pwm_pulse_width_to_level(pulse_width);
//    pwm_set_chan_level(slice_elevon, PWM_CHAN_B, level);
//}
//
//void pwm_set_right_motor(uint16_t pulse_width) {
//    uint16_t level = pwm_pulse_width_to_level(pulse_width);
//    pwm_set_chan_level(slice_motor, PWM_CHAN_A, level);
//}
//
//void pwm_set_left_motor(uint16_t pulse_width) {
//    uint16_t level = pwm_pulse_width_to_level(pulse_width);
//    pwm_set_chan_level(slice_motor, PWM_CHAN_B, level);
//}
