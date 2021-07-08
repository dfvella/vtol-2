#include "pwm.h"

static uint32_t counter_wrap;

// helper function for linear interpolation
static inline float interpolate(
    float val,
    float min_from,
    float max_from,
    float min_to,
    float max_to
) {
    return (((val - min_from) / (max_from - min_from)) * (max_to - min_to)) + min_to;
}

// convert PWM pulse width in microseconds to 32 bit integer to compare
// with the PWM counter
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

// Initialize gpio pins and PIO state machines
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

// Input is a number between -100 and 100.
// An input of -100 commands the servo to the min throw position
// An input of 100 commands the servo to the max thro position
void pwm_set_right_elevon(float input) {
    uint16_t pulse_width;

#   if PWM_REVERSE_RIGHT_ELEVON == 1
        input *= -1;
#   endif

    if (input < PWM_CEN_INPUT) {
        pulse_width = (uint16_t)interpolate(input,
            PWM_MIN_INPUT, PWM_CEN_INPUT,
            PWM_RIGHT_ELEVON_MIN, PWM_RIGHT_ELEVON_CEN
        );
    } else {
        pulse_width = (uint16_t)interpolate(input,
            PWM_CEN_INPUT, PWM_MAX_INPUT,
            PWM_RIGHT_ELEVON_CEN, PWM_RIGHT_ELEVON_MAX
        );
    }

    uint32_t level = pwm_pulse_width_to_level(pulse_width);
    pio_sm_put(pio0, PWM_SM_RIGHT_ELEVON, level);
}

// Input is a number between -100 and 100.
// An input of -100 commands the servo to the min throw position
// An input of 100 commands the servo to the max thro position
void pwm_set_left_elevon(float input) {
    uint16_t pulse_width;

#   if PWM_REVERSE_LEFT_ELEVON == 1
        input *= -1;
#   endif

    if (input < PWM_CEN_INPUT) {
        pulse_width = (uint16_t)interpolate(input,
            PWM_MIN_INPUT, PWM_CEN_INPUT,
            PWM_LEFT_ELEVON_MIN, PWM_LEFT_ELEVON_CEN
        );
    } else {
        pulse_width = (uint16_t)interpolate(input,
            PWM_CEN_INPUT, PWM_MAX_INPUT,
            PWM_LEFT_ELEVON_CEN, PWM_LEFT_ELEVON_MAX
        );
    }

    uint32_t level = pwm_pulse_width_to_level(pulse_width);
    pio_sm_put(pio0, PWM_SM_LEFT_ELEVON, level);
}

// Input is a number between -100 and 100.
// An input of -100 commands the motor to not turn
// An input of 100 commands the motor to turn at max speed
void pwm_set_right_motor(float input) {
    uint16_t pulse_width = (uint16_t)interpolate(input,
        PWM_MIN_INPUT, PWM_MAX_INPUT,
        PWM_RIGHT_MOTOR_MIN, PWM_RIGHT_MOTOR_MAX
    );

    uint32_t level = pwm_pulse_width_to_level(pulse_width);
    pio_sm_put(pio0, PWM_SM_RIGHT_MOTOR, level);
}

// Input is a number between -100 and 100.
// An input of -100 commands the motor to not turn
// An input of 100 commands the motor to turn at max speed
void pwm_set_left_motor(float input) {
    uint16_t pulse_width = (uint16_t)interpolate(input,
        PWM_MIN_INPUT, PWM_MAX_INPUT,
        PWM_LEFT_MOTOR_MIN, PWM_LEFT_MOTOR_MAX
    );

    uint32_t level = pwm_pulse_width_to_level(pulse_width);
    pio_sm_put(pio0, PWM_SM_LEFT_MOTOR, level);
}
