#include "pwm_schedular.h"

PWMSchedular::PWMSchedular() { }

void PWMSchedular::begin()
{
    for (uint8_t i = 0; i < NUM_DEVICES; ++i)
    {
        devices[i].pulsewidth = PWM_MID_PULSE;
    }

    devices[RIGHT_MOTOR].pin = RIGHT_MOTOR_PIN;
    devices[RIGHT_TILT].pin = RIGHT_TILT_PIN;
    devices[RIGHT_ALR].pin = RIGHT_ALR_PIN;
    devices[LEFT_MOTOR].pin = LEFT_MOTOR_PIN;
    devices[LEFT_TILT].pin = LEFT_TILT_PIN;
    devices[LEFT_ALR].pin = LEFT_ALR_PIN;
    devices[ELEVATOR].pin = ELEVATOR_PIN;

    pinMode(RIGHT_MOTOR_PIN, OUTPUT);
    pinMode(RIGHT_TILT_PIN, OUTPUT);
    pinMode(RIGHT_ALR_PIN, OUTPUT);
    pinMode(LEFT_MOTOR_PIN, OUTPUT);
    pinMode(LEFT_TILT_PIN, OUTPUT);
    pinMode(LEFT_ALR_PIN, OUTPUT);
    pinMode(ELEVATOR_PIN, OUTPUT);
}

void PWMSchedular::set_right_motor(uint16_t pulsewidth)
{
    devices[RIGHT_MOTOR].pulsewidth = constrain(pulsewidth,
        RIGHT_MOTOR_MIN_PULSE, RIGHT_MOTOR_MAX_PULSE);
}

void PWMSchedular::set_right_tilt(uint16_t pulsewidth)
{
    devices[RIGHT_TILT].pulsewidth = constrain(pulsewidth,
        RIGHT_TILT_MIN_PULSE, RIGHT_TILT_MAX_PULSE);
}

void PWMSchedular::set_right_aileron(uint16_t pulsewidth)
{
    devices[RIGHT_ALR].pulsewidth = constrain(pulsewidth,
        RIGHT_ALR_MIN_PULSE, RIGHT_ALR_MAX_PULSE);
}

void PWMSchedular::set_left_motor(uint16_t pulsewidth)
{
    devices[LEFT_MOTOR].pulsewidth = constrain(pulsewidth,
        LEFT_MOTOR_MIN_PULSE, LEFT_MOTOR_MAX_PULSE);
}

void PWMSchedular::set_left_tilt(uint16_t pulsewidth)
{
    devices[LEFT_TILT].pulsewidth = constrain(pulsewidth,
        LEFT_TILT_MIN_PULSE, LEFT_TILT_MAX_PULSE);
}

void PWMSchedular::set_left_aileron(uint16_t pulsewidth)
{
    devices[LEFT_ALR].pulsewidth = constrain(pulsewidth,
        LEFT_ALR_MIN_PULSE, LEFT_ALR_MAX_PULSE);
}

void PWMSchedular::set_elevator(uint16_t pulsewidth)
{
    devices[ELEVATOR].pulsewidth = constrain(pulsewidth,
        ELEVATOR_MIN_PULSE, ELEVATOR_MAX_PULSE);
}

uint16_t PWMSchedular::get_right_motor()
{
    return devices[RIGHT_MOTOR].pulsewidth;
}

uint16_t PWMSchedular::get_right_tilt()
{
    return devices[RIGHT_TILT].pulsewidth;
}

uint16_t PWMSchedular::get_right_aileron()
{
    return devices[RIGHT_ALR].pulsewidth;
}

uint16_t PWMSchedular::get_left_motor()
{
    return devices[LEFT_MOTOR].pulsewidth;
}

uint16_t PWMSchedular::get_left_tilt()
{
    return devices[LEFT_TILT].pulsewidth;
}

uint16_t PWMSchedular::get_left_aileron()
{
    return devices[LEFT_ALR].pulsewidth;
}

uint16_t PWMSchedular::get_elevator()
{
    return devices[ELEVATOR].pulsewidth;
}

void PWMSchedular::write()
{
    Device* sorted_devices = new Device[NUM_DEVICES];

    for (uint8_t i = 0; i < NUM_DEVICES; ++i)
    {
        sorted_devices[i] = devices[i];
    }

    for (uint8_t i = 0; i < NUM_DEVICES; ++i)
    {
        uint16_t min = i;

        for (uint8_t j = i; j < NUM_DEVICES; ++j)
        {
            if (sorted_devices[j].pulsewidth < sorted_devices[min].pulsewidth)
            {
                min = j;
            }
        }

        if (min != i)
        {
            swap(sorted_devices[i], sorted_devices[min]);
        }
    }

    for (uint8_t i = 0; i < NUM_DEVICES; ++i)
    {
        digitalWrite(sorted_devices[i].pin, HIGH);
        sorted_devices[i].timer = micros();

        unsigned long timer = micros();
        while (micros() - timer < PULSE_PADDING);
    }

    for (uint8_t i = 0; i < NUM_DEVICES; ++i)
    {
        while (micros() - sorted_devices[i].timer < sorted_devices[i].pulsewidth);

        digitalWrite(sorted_devices[i].pin, LOW);
    }

    delete[] sorted_devices;
}
