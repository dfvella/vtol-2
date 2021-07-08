#ifndef __AR610_H__
#define __AR610_H__

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*
 * Minimum difference between center stick and current stick input required for
 * ar610_get_<chan> functions to return the actual stick input. If threshold is
 * not reached, functions will return center stick.
 */
#define AR610_DEAD_STICK 10 /* units: microseconds */

#define AR610_REVERSE_THRO 0 /* set to 1 to flip sign of ar610_get_<chan> */
#define AR610_REVERSE_AILE 1 /* set to 1 to flip sign of ar610_get_<chan> */
#define AR610_REVERSE_ELEV 1 /* set to 1 to flip sign of ar610_get_<chan> */
#define AR610_REVERSE_RUDD 1 /* set to 1 to flip sign of ar610_get_<chan> */
#define AR610_REVERSE_GEAR 0 /* set to 1 to flip sign of ar610_get_<chan> */
#define AR610_REVERSE_AUX1 0 /* set to 1 to flip sign of ar610_get_<chan> */

#define AR610_MIN_VAL -100 /* min return value of ar610_get_<chan> */
#define AR610_MAX_VAL 100 /* max return value of ar610_get_<chan> */

#define AR610_CHANNEL_THRO 0
#define AR610_CHANNEL_AILE 1
#define AR610_CHANNEL_ELEV 2
#define AR610_CHANNEL_RUDD 3
#define AR610_CHANNEL_GEAR 4
#define AR610_CHANNEL_AUX1 5
#define AR610_NUM_CHANNELS 6

#define AR610_PWM_MIN_PULSEWIDTH 1000 /* units: microseconds */
#define AR610_PWM_CEN_PULSEWIDTH 1500 /* units: microseconds */
#define AR610_PWM_MAX_PULSEWIDTH 2000 /* units: microseconds */

#define AR610_THRO_PWM_MIN_PULSE 1096 /* units: microseconds */
#define AR610_THRO_PWM_MAX_PULSE 1900 /* units: microseconds */

#define AR610_AILE_PWM_MIN_PULSE 1096 /* units: microseconds */
#define AR610_AILE_PWM_CEN_PULSE 1498 /* units: microseconds */
#define AR610_AILE_PWM_MAX_PULSE 1900 /* units: microseconds */

#define AR610_ELEV_PWM_MIN_PULSE 1096 /* units: microseconds */
#define AR610_ELEV_PWM_CEN_PULSE 1498 /* units: microseconds */
#define AR610_ELEV_PWM_MAX_PULSE 1900 /* units: microseconds */

#define AR610_RUDD_PWM_MIN_PULSE 1096 /* units: microseconds */
#define AR610_RUDD_PWM_CEN_PULSE 1498 /* units: microseconds */
#define AR610_RUDD_PWM_MAX_PULSE 1900 /* units: microseconds */

#define AR610_GEAR_PWM_MIN_PULSE 1096 /* units: microseconds */
#define AR610_GEAR_PWM_CEN_PULSE 1498 /* units: microseconds */
#define AR610_GEAR_PWM_MAX_PULSE 1900 /* units: microseconds */

#define AR610_AUX1_PWM_MIN_PULSE 1096 /* units: microseconds */
#define AR610_AUX1_PWM_CEN_PULSE 1498 /* units: microseconds */
#define AR610_AUX1_PWM_MAX_PULSE 1900 /* units: microseconds */

/*
 * Divides system clock to drive pwm hardware counters
 */
#define AR610_PWM_CLOCK_DIVIDER 100.0f

/*
 * Minimum change in measured pwm pulsewidth required to update return value
 * of ar610_get_<channel>(). For noise reduction.
 */
#define AR610_PWM_STEP_FILTER 2


/*
 * object for containing ar610 state
 */
struct ar610_inst {
    uint pin[AR610_NUM_CHANNELS];
    uint slice[AR610_NUM_CHANNELS];
    uint16_t prev1[AR610_NUM_CHANNELS];
    uint16_t prev2[AR610_NUM_CHANNELS];
};

/*
 * object for containing ar610 state
 */
typedef struct ar610_inst ar610_inst_t;

/*
 * Initialize ar610 object, pwm hardware, and gpio pins
 */
void ar610_init(ar610_inst_t* inst,
    uint thro_pin, uint aile_pin, uint elev_pin,
    uint rudd_pin, uint gear_pin, uint aux1_pin);

/*
 * Updates the state of the ar610 object. Reads all pwm channels.
 * This function must be called between 2 and 50Hz.
 */
void ar610_update_state(ar610_inst_t* inst);

/*
 * Returns 1 if the ar610 is connected to a transmitter.
 * This function requires ar610_update_state() to be called
 * between 2 and 50Hz for its return value to be valid.
 */
uint8_t ar610_is_connected(ar610_inst_t* inst);

/*
 * Returns the pwm pulsewidth of the throttle channel in microseconds.
 * This function requires ar610_update_state() to be called
 * between 2 and 50Hz for its return value to be valid.
 */
uint16_t ar610_get_thro_us(ar610_inst_t* inst);

/*
 * Returns the pwm pulsewidth of the aileron channel in microseconds.
 * This function requires ar610_update_state() to be called
 * between 2 and 50Hz for its return value to be valid.
 */
uint16_t ar610_get_aile_us(ar610_inst_t* inst);

/*
 * Returns the pwm pulsewidth of the elevator channel in microseconds.
 * This function requires ar610_update_state() to be called
 * between 2 and 50Hz for its return value to be valid.
 */
uint16_t ar610_get_elev_us(ar610_inst_t* inst);

/*
 * Returns the pwm pulsewidth of the rudder channel in microseconds.
 * This function requires ar610_update_state() to be called
 * between 2 and 50Hz for its return value to be valid.
 */
uint16_t ar610_get_rudd_us(ar610_inst_t* inst);

/*
 * Returns the pwm pulsewidth of the gear channel in microseconds.
 * This function requires ar610_update_state() to be called
 * between 2 and 50Hz for its return value to be valid.
 */
uint16_t ar610_get_gear_us(ar610_inst_t* inst);

/*
 * Returns the pwm pulsewidth of the aux1 channel in microseconds.
 * This function requires ar610_update_state() to be called
 * between 2 and 50Hz for its return value to be valid.
 */
uint16_t ar610_get_aux1_us(ar610_inst_t* inst);

/*
 * Returns the input of the throttle channel as a number between -100 and 100.
 * This function requires ar610_update_state() to be called between 2 and 50Hz
 * for its return value to be valid. Additionally, this function requires
 * AR610_PWM_THRO_MIN_PULSE and AR610_PWM_THRO_MAX_PULSE to be accurate.
 */
float ar610_get_thro(ar610_inst_t* inst);

/*
 * Returns the input of the aileron channel as a number between -100 and 100.
 * This function requires ar610_update_state() to be called between 2 and 50Hz
 * for its return value to be valid. Additionally, this function requires
 * AR610_PWM_AILE_MIN_PULSE, AR610_PWM_AILE_CEN_PULSE, and
 * AR610_PWM_AILE_MAX_PULSE to be accurate.
 */
float ar610_get_aile(ar610_inst_t* inst);

/*
 * Returns the input of the elevator channel as a number between -100 and 100.
 * This function requires ar610_update_state() to be called between 2 and 50Hz
 * for its return value to be valid. Additionally, this function requires
 * AR610_PWM_ELEV_MIN_PULSE, AR610_PWM_ELEV_CEN_PULSE, and
 * AR610_PWM_ELEV_MAX_PULSE to be accurate.
 */
float ar610_get_elev(ar610_inst_t* inst);

/*
 * Returns the input of the rudder channel as a number between -100 and 100.
 * This function requires ar610_update_state() to be called between 2 and 50Hz
 * for its return value to be valid. Additionally, this function requires
 * AR610_PWM_RUDD_MIN_PULSE, AR610_PWM_RUDD_CEN_PULSE, and
 * AR610_PWM_RUDD_MAX_PULSE to be accurate.
 */
float ar610_get_rudd(ar610_inst_t* inst);

/*
 * Returns the input of the gear channel as a number between -100 and 100.
 * This function requires ar610_update_state() to be called between 2 and 50Hz
 * for its return value to be valid. Additionally, this function requires
 * AR610_PWM_GEAR_MIN_PULSE, AR610_PWM_GEAR_CEN_PULSE, and
 * AR610_PWM_GEAR_MAX_PULSE to be accurate.
 */
float ar610_get_gear(ar610_inst_t* inst);

/*
 * Returns the input of the gear aux1 as a number between -100 and 100.
 * This function requires ar610_update_state() to be called between 2 and 50Hz
 * for its return value to be valid. Additionally, this function requires
 * AR610_PWM_AUX1_MIN_PULSE, AR610_PWM_AUX1_CEN_PULSE, and
 * AR610_PWM_AUX1_MAX_PULSE to be accurate.
 */
float ar610_get_aux1(ar610_inst_t* inst);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __AR610_H__ */
