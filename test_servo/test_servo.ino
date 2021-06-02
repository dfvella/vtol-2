#include <Arduino.h>

#include "pwm_schedular.h"

//#define DO_PRINTS

PWMSchedular pwm;

unsigned long timer = 0;

void setup()
{
    #ifdef DO_PRINTS
    Serial.begin(9600);
    #endif

    pwm.begin();

    timer = micros();
}

void loop()
{
    static uint16_t signal = 1000;
    static uint32_t counter = 0;

    if (counter >= 500)
        counter = 0;

    if (counter < 250)
        signal += 5;
    else
        signal -= 5;

    pwm.set_right_motor(RIGHT_MOTOR_MIN_PULSE);
    pwm.set_left_motor(LEFT_MOTOR_MIN_PULSE);

    pwm.set_right_tilt(signal);
    pwm.set_left_tilt(signal);
    pwm.set_right_aileron(signal);
    pwm.set_left_aileron(signal);
    pwm.set_elevator(signal);

    #ifdef DO_PRINTS
    Serial.print(pwm.get_right_motor());
    Serial.print(' ');
    Serial.print(pwm.get_right_tilt());
    Serial.print(' ');
    Serial.print(pwm.get_right_aileron());
    Serial.print(' ');
    Serial.print(pwm.get_left_motor());
    Serial.print(' ');
    Serial.print(pwm.get_left_tilt());
    Serial.print(' ');
    Serial.print(pwm.get_left_aileron());
    Serial.print(' ');
    Serial.print(pwm.get_elevator());
    Serial.println();
    #endif

    pwm.write();

    ++counter;

    while (micros() - timer < 20000);
    timer = micros();
}
