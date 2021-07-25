#ifndef __PWM_H__
#define __PWM_H__

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"

#include "../build/src/pwm.S.h"

#include "constants.h"

#define PWM_REVERSE_RIGHT_ELEVON 0 // set as 1 to reverse right elevon servo
#define PWM_REVERSE_LEFT_ELEVON 1 // set as 1 to reverse left elevon servo

#define PWM_RIGHT_ELEVON_MIN 1200 // minimum pwm pulse width in microseconds
#define PWM_RIGHT_ELEVON_CEN 1500 // center pwm pulse width in microseconds
#define PWM_RIGHT_ELEVON_MAX 1800 // maximum pwm pulse width in microseconds

#define PWM_LEFT_ELEVON_MIN 1200 // minimum pwm pulse width in microseconds
#define PWM_LEFT_ELEVON_CEN 1500 // center pwm pulse width in microseconds
#define PWM_LEFT_ELEVON_MAX 1800 // maximum pwm pulse width in microseconds

#define PWM_RIGHT_MOTOR_MIN 1100 // minimum pwm pulse width in microseconds
#define PWM_RIGHT_MOTOR_MAX 1900 // maximumm pwm pulse width in microseconds

#define PWM_LEFT_MOTOR_MIN 1100 // minimum pwm pulse width in microseconds
#define PWM_LEFT_MOTOR_MAX 1900 // maximum pwm pulse width in microseconds

#define PWM_MIN_INPUT -100 // minimum val to pass as input to pwm_set functions
#define PWM_CEN_INPUT 0 // center value to pass as input to pwm_set functions
#define PWM_MAX_INPUT 100 // maximum val to pass as input to pwm_set functions

// number of cycles required for the PIO state machine to compare the PWM
// counter against the level and decrement the counter
#define PWM_CYCLES 3
#define PWM_FREQUENCY 50 // units: Hertz
#define PWM_PERIOD_US 20000.0f // units: microseconds

#define PWM_SM_RIGHT_ELEVON 0 // PIO state machine assignment
#define PWM_SM_LEFT_ELEVON  1 // PIO state machine assignment
#define PWM_SM_RIGHT_MOTOR  2 // PIO state machine assignment
#define PWM_SM_LEFT_MOTOR   3 // PIO state machine assignment

#define PWM_PIO_RIGHT_ELEVON pio0 // PIO block assignment
#define PWM_PIO_LEFT_ELEVON pio0 // PIO block assignment
#define PWM_PIO_RIGHT_MOTOR pio0 // PIO block assignment
#define PWM_PIO_LEFT_MOTOR pio0 // PIO block assignment

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

// call at the start of main or motor controllers will enter a failure state
void pwm_esc_patch();

// Initialize gpio pins and PIO state machines
void pwm_init_all_outputs();

// Input is a number between -100 and 100.
// An input of -100 commands the servo to the min throw position
// An input of 100 commands the servo to the max thro position
void pwm_set_right_elevon(float input);

// Input is a number between -100 and 100.
// An input of -100 commands the servo to the min throw position
// An input of 100 commands the servo to the max thro position
void pwm_set_left_elevon(float input);

// Input is a number between -100 and 100.
// An input of -100 commands the motor to not turn
// An input of 100 commands the motor to turn at max speed
void pwm_set_right_motor(float input);

// Input is a number between -100 and 100.
// An input of -100 commands the motor to not turn
// An input of 100 commands the motor to turn at max speed
void pwm_set_left_motor(float input);

// Set the PWM hardware to 0% dutycycle on all channels
void pwm_disable_all_outputs();

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // __PWM_H__
