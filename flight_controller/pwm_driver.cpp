#include "pwm_driver.h"

Pwm::Pwm() { }

void Pwm::begin()
{
    pinMode(RIGHT_AILERON_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_PIN, OUTPUT);
    pinMode(LEFT_AILERON_PIN, OUTPUT);
    pinMode(LEFT_MOTOR_PIN, OUTPUT);

    // clear clock divider config bits in timer config registers
    TCCR1B &= 0b11111000;
    TCCR2B &= 0b11111000;

    // config clock divider
    TCCR1B |= TIMER_1_CLOCK_DIVIDER;
    TCCR2B |= TIMER_2_CLOCK_DIVIDER;
}

void Pwm::set_right_aileron(uint16_t pulsewidth)
{
    pulsewidth = constrain(pulsewidth, RIGHT_AILERON_MIN_PULSE, RIGHT_AILERON_MAX_PULSE);
    analogWrite(RIGHT_AILERON_PIN, get_cycles(pulsewidth));
}

void Pwm::set_right_motor(uint16_t pulsewidth)
{
    pulsewidth = constrain(pulsewidth, RIGHT_MOTOR_MIN_PULSE, RIGHT_MOTOR_MAX_PULSE);
    analogWrite(RIGHT_MOTOR_PIN, get_cycles(pulsewidth));
}

void Pwm::set_left_aileron(uint16_t pulsewidth)
{
    pulsewidth = constrain(pulsewidth, LEFT_AILERON_MIN_PULSE, LEFT_AILERON_MAX_PULSE);
    analogWrite(LEFT_AILERON_PIN, get_cycles(pulsewidth));
}

void Pwm::set_left_motor(uint16_t pulsewidth)
{
    pulsewidth = constrain(pulsewidth, LEFT_MOTOR_MIN_PULSE, LEFT_MOTOR_MAX_PULSE);
    analogWrite(LEFT_MOTOR_PIN, get_cycles(pulsewidth));
}

int Pwm::get_cycles(uint16_t pulsewidth)
{
    return (uint8_t)((pulsewidth / 16319.0f) * 255) + 20;
}
