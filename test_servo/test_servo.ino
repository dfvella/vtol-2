#include <Arduino.h>

#include "pwm_driver.h"

//#define COMMAND_MOTORS

Pwm pwm;

void setup()
{
    pwm.begin();
}

void loop()
{
    pwm.set_right_aileron(RIGHT_AILERON_MIN_PULSE);
    pwm.set_left_aileron(LEFT_AILERON_MIN_PULSE);

    pwm.set_right_motor(RIGHT_MOTOR_MIN_PULSE);
    pwm.set_left_motor(LEFT_MOTOR_MIN_PULSE);

    delay(1000);

    pwm.set_right_aileron(RIGHT_AILERON_MAX_PULSE);
    pwm.set_left_aileron(LEFT_AILERON_MAX_PULSE);

    #ifdef COMMAND_MOTORS
    pwm.set_right_motor(RIGHT_MOTOR_MAX_PULSE);
    pwm.set_left_motor(LEFT_MOTOR_MAX_PULSE);
    #else
    pwm.set_right_motor(RIGHT_MOTOR_MIN_PULSE);
    pwm.set_left_motor(LEFT_MOTOR_MIN_PULSE);
    #endif

    delay(1000);
}
