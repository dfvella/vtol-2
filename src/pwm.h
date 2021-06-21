#ifndef __PWM_H__
#define __PWM_H__

#include "pico/stdlib.h"
#include "hardware/clocks.h"
// #include "hardware/gpio.h"
// #include "hardware/pwm.h"

#include "hardware/pio.h"
#include "pwm.s.h"

#include "constants.h"

#define PWM_CYCLES 3
#define PWM_FREQUENCY 50
#define PWM_CLOCK_DIVIDER 255.0f
#define PWM_PERIOD_US 20000.0f

#define PWM_SM_RIGHT_ELEVON 0
#define PWM_SM_LEFT_ELEVON  1
#define PWM_SM_RIGHT_MOTOR  2
#define PWM_SM_LEFT_MOTOR   3

#define PWM_PIO_RIGHT_ELEVON pio0
#define PWM_PIO_LEFT_ELEVON pio0
#define PWM_PIO_RIGHT_MOTOR pio0
#define PWM_PIO_LEFT_MOTOR pio0

void pwm_init_all_outputs();

void pwm_set_right_elevon(uint16_t pulse_width);
void pwm_set_left_elevon(uint16_t pulse_width);
void pwm_set_right_motor(uint16_t pulse_width);
void pwm_set_left_motor(uint16_t pulse_width);

#endif // __PWM_H__
