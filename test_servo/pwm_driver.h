#ifndef PWM_DRIVER_H
#define PWM_DRIVER_H

// uses builtin PWM hardware to command to servos and motor controllers
// asynchronous from main thread

#include <Arduino.h>
#include <avr/io.h>

// hack to make intelisense happy
#ifndef _AVR_IOXXX_H_
#include <avr/iom328p.h>
#endif

#define RIGHT_MOTOR_PIN 3
#define RIGHT_AILERON_PIN 9
#define LEFT_MOTOR_PIN 10
#define LEFT_AILERON_PIN 11

#define RIGHT_MOTOR_MIN_PULSE 1100
#define RIGHT_MOTOR_MAX_PULSE 1900

#define RIGHT_AILERON_MIN_PULSE 1210
#define RIGHT_AILERON_MAX_PULSE 1850

#define LEFT_MOTOR_MIN_PULSE 1100
#define LEFT_MOTOR_MAX_PULSE 1900

#define LEFT_AILERON_MIN_PULSE 1200
#define LEFT_AILERON_MAX_PULSE 1710

//#define TIMER_1_CLOCK_DIVIDER 0b00000001 // counter wrap freq: 31372 Hz
//#define TIMER_1_CLOCK_DIVIDER 0b00000010 // counter wrap freq: 3921 Hz
//#define TIMER_1_CLOCK_DIVIDER 0b00000011 // counter wrap freq: 490 Hz
#define TIMER_1_CLOCK_DIVIDER 0b00000100 // counter wrap freq: 122 Hz
//#define TIMER_1_CLOCK_DIVIDER 0b00000101 // counter wrap freq: 30 Hz

//#define TIMER_2_CLOCK_DIVIDER 0b00000001 // counter wrap freq: 31372 Hz
//#define TIMER_2_CLOCK_DIVIDER 0b00000010 // counter wrap freq: 3921 Hz
//#define TIMER_2_CLOCK_DIVIDER 0b00000011 // counter wrap freq: 980 Hz
//#define TIMER_2_CLOCK_DIVIDER 0b00000100 // counter wrap freq: 490 Hz
//#define TIMER_2_CLOCK_DIVIDER 0b00000101 // counter wrap freq: 245 Hz
#define TIMER_2_CLOCK_DIVIDER 0b00000110 // counter wrap freq: 122 Hz
//#define TIMER_2_CLOCK_DIVIDER 0b00000111 // counter wrap freq: 30 Hz

class Pwm
{
public:
    Pwm();

    void begin();

    void set_right_aileron(uint16_t pulsewidth); // pulsewidth units: microseconds
    void set_right_motor(uint16_t pulsewidth); // pulsewdidth units: microseconds
    void set_left_aileron(uint16_t pulsewidth); // pulsewidth units: microseconds
    void set_left_motor(uint16_t pulsewidth); // pulsewidth units: microseconds

private:
    int get_cycles(uint16_t pulsewidth);
};

#endif // PWM_DRIVER_H
