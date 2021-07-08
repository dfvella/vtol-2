#include "ar610.h"

/*
 * returns the absolute value of val
 */
static inline uint16_t abs16u(int16_t val) {
    return val < 0 ? 0 - val : val;
}

/*
 * If dead stick threshold is not reached, return center stick
 */
static inline uint16_t apply_dead_stick(uint16_t input, uint16_t center) {
    if (abs16u((int16_t)input - (int16_t)center) < AR610_DEAD_STICK) {
        return center;
    } else {
        return input;
    }
}

/*
 * linearly interpolates val from (min_from, max_from) to (min_to, max_to)
 */
static inline float interpolate(
    float val,
    float min_from,
    float max_from,
    float min_to,
    float max_to
) {
    return (((val - min_from) / (max_from - min_from)) * (max_to - min_to)) + min_to;
}

/*
 * Initialize ar610 object, pwm hardware, and gpio pins
 */
void ar610_init(ar610_inst_t* inst,
    uint pin_thro, uint pin_aile, uint pin_elev,
    uint pin_rudd, uint pin_gear, uint pin_aux1) {

    inst->pin[AR610_CHANNEL_THRO] = pin_thro;
    inst->pin[AR610_CHANNEL_AILE] = pin_aile;
    inst->pin[AR610_CHANNEL_ELEV] = pin_elev;
    inst->pin[AR610_CHANNEL_RUDD] = pin_rudd;
    inst->pin[AR610_CHANNEL_GEAR] = pin_gear;
    inst->pin[AR610_CHANNEL_AUX1] = pin_aux1;

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_HIGH);
    pwm_config_set_clkdiv(&cfg, AR610_PWM_CLOCK_DIVIDER);

    for (uint8_t i = 0; i < AR610_NUM_CHANNELS; ++i) {
        inst->slice[i] = pwm_gpio_to_slice_num(inst->pin[i]);

        pwm_init(inst->slice[i], &cfg, false);
        gpio_set_function(inst->pin[i], GPIO_FUNC_PWM);
        pwm_set_enabled(inst->slice[i], true);

        inst->prev1[i] = 0;
        inst->prev2[i] = 0;
    }
}

/*
 * Updates the state of the ar610 object. Reads all pwm channels.
 * This function must be called between 2 and 50Hz.
 */
void ar610_update_state(ar610_inst_t* inst) {
    for (uint8_t chan = 0; chan < AR610_NUM_CHANNELS; ++chan) {
        uint16_t count = pwm_get_counter(inst->slice[chan]);
        pwm_set_counter(inst->slice[chan], 0);

        uint16_t curr = (count * AR610_PWM_CLOCK_DIVIDER * 1000000) / clock_get_hz(clk_sys);

        uint16_t diff1 = abs16u(curr + inst->prev1[chan] - inst->prev2[chan]);
        uint16_t diff2 = abs16u(inst->prev1[chan] - inst->prev2[chan]);

        /* logic to check if the counter was read in the middle of a pulse and correct it */
        if (diff1 < diff2 && curr + inst->prev1[chan] < AR610_PWM_MAX_PULSEWIDTH) {
            if (diff1 > AR610_PWM_STEP_FILTER)
                inst->prev2[chan] = curr + inst->prev1[chan];

            inst->prev1[chan] = curr + inst->prev1[chan];
        } else {
            if (diff2 > AR610_PWM_STEP_FILTER) 
                inst->prev2[chan] = inst->prev1[chan];

            inst->prev1[chan] = curr;
        }
    }
}

/*
 * Returns 1 if the ar610 is connected to a transmitter.
 * This function requires ar610_update_state() to be called
 * between 2 and 50Hz for its return value to be valid.
 */
uint8_t ar610_is_connected(ar610_inst_t* inst) {
    return inst->prev2[AR610_CHANNEL_AILE] ||
           inst->prev2[AR610_CHANNEL_ELEV] ||
           inst->prev2[AR610_CHANNEL_RUDD] ||
           inst->prev2[AR610_CHANNEL_GEAR] ||
           inst->prev2[AR610_CHANNEL_AUX1];
}

/*
 * Returns the pwm pulsewidth of the throttle channel in microseconds.
 * This function requires ar610_update_state() to be called
 * between 2 and 50Hz for its return value to be valid.
 */
uint16_t ar610_get_thro_us(ar610_inst_t* inst) {
    uint16_t pulse = inst->prev2[AR610_CHANNEL_THRO];
    return apply_dead_stick(pulse, AR610_THRO_PWM_MIN_PULSE);
}

/*
 * Returns the pwm pulsewidth of the aileron channel in microseconds.
 * This function requires ar610_update_state() to be called
 * between 2 and 50Hz for its return value to be valid.
 */
uint16_t ar610_get_aile_us(ar610_inst_t* inst) {
    uint16_t pulse = inst->prev2[AR610_CHANNEL_AILE];
    return apply_dead_stick(pulse, AR610_AILE_PWM_CEN_PULSE);
}

/*
 * Returns the pwm pulsewidth of the elevator channel in microseconds.
 * This function requires ar610_update_state() to be called
 * between 2 and 50Hz for its return value to be valid.
 */
uint16_t ar610_get_elev_us(ar610_inst_t* inst) {
    uint16_t pulse = inst->prev2[AR610_CHANNEL_ELEV];
    return apply_dead_stick(pulse, AR610_ELEV_PWM_CEN_PULSE);
}

/*
 * Returns the pwm pulsewidth of the rudder channel in microseconds.
 * This function requires ar610_update_state() to be called
 * between 2 and 50Hz for its return value to be valid.
 */
uint16_t ar610_get_rudd_us(ar610_inst_t* inst) {
    uint16_t pulse = inst->prev2[AR610_CHANNEL_RUDD];
    return apply_dead_stick(pulse, AR610_RUDD_PWM_CEN_PULSE);
}

/*
 * Returns the pwm pulsewidth of the gear channel in microseconds.
 * This function requires ar610_update_state() to be called
 * between 2 and 50Hz for its return value to be valid.
 */
uint16_t ar610_get_gear_us(ar610_inst_t* inst) {
    uint16_t pulse = inst->prev2[AR610_CHANNEL_GEAR];
    return apply_dead_stick(pulse, AR610_GEAR_PWM_CEN_PULSE);
}

/*
 * Returns the pwm pulsewidth of the aux1 channel in microseconds.
 * This function requires ar610_update_state() to be called
 * between 2 and 50Hz for its return value to be valid.
 */
uint16_t ar610_get_aux1_us(ar610_inst_t* inst) {
    uint16_t pulse = inst->prev2[AR610_CHANNEL_AUX1];
    return apply_dead_stick(pulse, AR610_AUX1_PWM_CEN_PULSE);
}

/*
 * Returns the input of the throttle channel as a number between -100 and 100.
 * This function requires ar610_update_state() to be called between 2 and 50Hz
 * for its return value to be valid. Additionally, this function requires
 * AR610_PWM_THRO_MIN_PULSE and AR610_PWM_THRO_MAX_PULSE to be accurate.
 */
float ar610_get_thro(ar610_inst_t* inst) {
    float res = interpolate(
        ar610_get_thro_us(inst),
        AR610_THRO_PWM_MIN_PULSE,
        AR610_THRO_PWM_MAX_PULSE,
        AR610_MIN_VAL,
        AR610_MAX_VAL
    );

#   if AR610_REVERSE_THRO == 1
        res *= -1;
#   endif

    return res;
}

/*
 * Returns the input of the aileron channel as a number between -100 and 100.
 * This function requires ar610_update_state() to be called between 2 and 50Hz
 * for its return value to be valid. Additionally, this function requires
 * AR610_PWM_AILE_MIN_PULSE, AR610_PWM_AILE_CEN_PULSE, and
 * AR610_PWM_AILE_MAX_PULSE to be accurate.
 */
float ar610_get_aile(ar610_inst_t* inst) {
    float res = interpolate(
        ar610_get_aile_us(inst),
        AR610_AILE_PWM_MIN_PULSE,
        AR610_AILE_PWM_MAX_PULSE,
        AR610_MIN_VAL,
        AR610_MAX_VAL
    );

#   if AR610_REVERSE_AILE == 1
        res *= -1;
#   endif

    return res;
}

/*
 * Returns the input of the elevator channel as a number between -100 and 100.
 * This function requires ar610_update_state() to be called between 2 and 50Hz
 * for its return value to be valid. Additionally, this function requires
 * AR610_PWM_ELEV_MIN_PULSE, AR610_PWM_ELEV_CEN_PULSE, and
 * AR610_PWM_ELEV_MAX_PULSE to be accurate.
 */
float ar610_get_elev(ar610_inst_t* inst) {
    float res = interpolate(
        ar610_get_elev_us(inst),
        AR610_ELEV_PWM_MIN_PULSE,
        AR610_ELEV_PWM_MAX_PULSE,
        AR610_MIN_VAL,
        AR610_MAX_VAL
    );

#   if AR610_REVERSE_ELEV == 1
        res *= -1;
#   endif

    return res;
}

/*
 * Returns the input of the rudder channel as a number between -100 and 100.
 * This function requires ar610_update_state() to be called between 2 and 50Hz
 * for its return value to be valid. Additionally, this function requires
 * AR610_PWM_RUDD_MIN_PULSE, AR610_PWM_RUDD_CEN_PULSE, and
 * AR610_PWM_RUDD_MAX_PULSE to be accurate.
 */
float ar610_get_rudd(ar610_inst_t* inst) {
    float res = interpolate(
        ar610_get_rudd_us(inst),
        AR610_RUDD_PWM_MIN_PULSE,
        AR610_RUDD_PWM_MAX_PULSE,
        AR610_MIN_VAL,
        AR610_MAX_VAL
    );

#   if AR610_REVERSE_RUDD == 1
        res *= -1;
#   endif

    return res;
}

/*
 * Returns the input of the gear channel as a number between -100 and 100.
 * This function requires ar610_update_state() to be called between 2 and 50Hz
 * for its return value to be valid. Additionally, this function requires
 * AR610_PWM_GEAR_MIN_PULSE, AR610_PWM_GEAR_CEN_PULSE, and
 * AR610_PWM_GEAR_MAX_PULSE to be accurate.
 */
float ar610_get_gear(ar610_inst_t* inst) {
    float res = interpolate(
        ar610_get_gear_us(inst),
        AR610_GEAR_PWM_MIN_PULSE,
        AR610_GEAR_PWM_MAX_PULSE,
        AR610_MIN_VAL,
        AR610_MAX_VAL
    );

#   if AR610_REVERSE_GEAR == 1
        res *= -1;
#   endif

    return res;
}

/*
 * Returns the input of the gear aux1 as a number between -100 and 100.
 * This function requires ar610_update_state() to be called between 2 and 50Hz
 * for its return value to be valid. Additionally, this function requires
 * AR610_PWM_AUX1_MIN_PULSE, AR610_PWM_AUX1_CEN_PULSE, and
 * AR610_PWM_AUX1_MAX_PULSE to be accurate.
 */
float ar610_get_aux1(ar610_inst_t* inst) {
    float res = interpolate(
        ar610_get_aux1_us(inst),
        AR610_AUX1_PWM_MIN_PULSE,
        AR610_AUX1_PWM_MAX_PULSE,
        AR610_MIN_VAL,
        AR610_MAX_VAL
    );

#   if AR610_REVERSE_AUX1 == 1
        res *= -1;
#   endif

    return res;
}
