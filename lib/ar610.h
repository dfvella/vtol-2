#ifndef __AR610_H__
#define __AR610_H__

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define AR610_CHANNEL_THRO 0
#define AR610_CHANNEL_AILE 1
#define AR610_CHANNEL_ELEV 2
#define AR610_CHANNEL_RUDD 3
#define AR610_CHANNEL_GEAR 4
#define AR610_CHANNEL_AUX1 5
#define AR610_NUM_CHANNELS 6

#define AR610_PWM_MIN_PULSEWIDTH 1000
#define AR610_PWM_MID_PULSEWIDTH 1500
#define AR610_PWM_MAX_PULSEWIDTH 2000

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
uint16_t ar610_get_thro(ar610_inst_t* inst);

/*
 * Returns the pwm pulsewidth of the aileron channel in microseconds.
 * This function requires ar610_update_state() to be called
 * between 2 and 50Hz for its return value to be valid.
 */
uint16_t ar610_get_aile(ar610_inst_t* inst);

/*
 * Returns the pwm pulsewidth of the elevator channel in microseconds.
 * This function requires ar610_update_state() to be called
 * between 2 and 50Hz for its return value to be valid.
 */
uint16_t ar610_get_elev(ar610_inst_t* inst);

/*
 * Returns the pwm pulsewidth of the rudder channel in microseconds.
 * This function requires ar610_update_state() to be called
 * between 2 and 50Hz for its return value to be valid.
 */
uint16_t ar610_get_rudd(ar610_inst_t* inst);

/*
 * Returns the pwm pulsewidth of the gear channel in microseconds.
 * This function requires ar610_update_state() to be called
 * between 2 and 50Hz for its return value to be valid.
 */
uint16_t ar610_get_gear(ar610_inst_t* inst);

/*
 * Returns the pwm pulsewidth of the aux1 channel in microseconds.
 * This function requires ar610_update_state() to be called
 * between 2 and 50Hz for its return value to be valid.
 */
uint16_t ar610_get_aux1(ar610_inst_t* inst);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __AR610_H__ */
