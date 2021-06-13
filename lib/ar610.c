#include "ar610.h"

/*
 * returns the absolute value of val
 */
static inline uint16_t abs16(int16_t val) {
    return val < 0 ? 0 - val : val;
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

        uint16_t diff1 = abs16(curr + inst->prev1[chan] - inst->prev2[chan]);
        uint16_t diff2 = abs16(inst->prev1[chan] - inst->prev2[chan]);

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
uint16_t ar610_get_thro(ar610_inst_t* inst) {
    return inst->prev2[AR610_CHANNEL_THRO];
}

/*
 * Returns the pwm pulsewidth of the aileron channel in microseconds.
 * This function requires ar610_update_state() to be called
 * between 2 and 50Hz for its return value to be valid.
 */
uint16_t ar610_get_aile(ar610_inst_t* inst) {
    return inst->prev2[AR610_CHANNEL_AILE];
}

/*
 * Returns the pwm pulsewidth of the elevator channel in microseconds.
 * This function requires ar610_update_state() to be called
 * between 2 and 50Hz for its return value to be valid.
 */
uint16_t ar610_get_elev(ar610_inst_t* inst) {
    return inst->prev2[AR610_CHANNEL_ELEV];
}

/*
 * Returns the pwm pulsewidth of the rudder channel in microseconds.
 * This function requires ar610_update_state() to be called
 * between 2 and 50Hz for its return value to be valid.
 */
uint16_t ar610_get_rudd(ar610_inst_t* inst) {
    return inst->prev2[AR610_CHANNEL_RUDD];
}

/*
 * Returns the pwm pulsewidth of the gear channel in microseconds.
 * This function requires ar610_update_state() to be called
 * between 2 and 50Hz for its return value to be valid.
 */
uint16_t ar610_get_gear(ar610_inst_t* inst) {
    return inst->prev2[AR610_CHANNEL_GEAR];
}

/*
 * Returns the pwm pulsewidth of the aux1 channel in microseconds.
 * This function requires ar610_update_state() to be called
 * between 2 and 50Hz for its return value to be valid.
 */
uint16_t ar610_get_aux1(ar610_inst_t* inst) {
    return inst->prev2[AR610_CHANNEL_AUX1];
}
