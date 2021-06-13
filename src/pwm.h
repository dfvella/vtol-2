#ifndef __PWM_H__
#define __PWM_H__

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#define RIGHT_ELEVON_PIN 12
#define LEFT_ELEVON_PIN 13
#define RIGHT_MOTOR_PIN 14
#define LEFT_MOTOR_PIN 15

#define PWM_FREQUENCY 50.0f
#define PWM_CLOCK_DIVIDER 255.0f

// Initialize PWM hardware
void pwm_init_all_outputs();

void pwm_set_right_elevon(uint16_t pulse_width);
void pwm_set_left_elevon(uint16_t pulse_width);
void pwm_set_right_motor(uint16_t pulse_width);
void pwm_set_left_motor(uint16_t pulse_width);

#endif // __PWM_H__
