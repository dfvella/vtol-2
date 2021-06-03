#ifndef PWM_SCHEDULAR_H
#define PWM_SCHEDULAR_H

#include <Arduino.h>

// pin assignments
#define RIGHT_MOTOR_PIN 3
#define RIGHT_ALR_PIN 9
#define LEFT_MOTOR_PIN 10
#define LEFT_ALR_PIN 11

#define RIGHT_MOTOR_MIN_PULSE 1100
#define RIGHT_MOTOR_MAX_PULSE 1900

#define RIGHT_ALR_MIN_PULSE 1000
#define RIGHT_ALR_MAX_PULSE 2000

#define LEFT_MOTOR_MIN_PULSE 1100
#define LEFT_MOTOR_MAX_PULSE 1900

#define LEFT_ALR_MIN_PULSE 1000
#define LEFT_ALR_MAX_PULSE 2000


class PWMSchedular
{
public:
    PWMSchedular();

    void begin();

    void set_right_motor(uint16_t pulsewidth);
    void set_right_aileron(uint16_t pulsewidth);
    void set_left_motor(uint16_t pulsewidth);
    void set_left_aileron(uint16_t pulsewidth);

    uint16_t get_right_motor();
    uint16_t get_right_aileron();
    uint16_t get_left_motor();
    uint16_t get_left_aileron();

    static constexpr uint16_t PWM_MIN_PULSE = 1000;
    static constexpr uint16_t PWM_MID_PULSE = 1500;
    static constexpr uint16_t PWM_MAX_PULSE = 2000;

    static constexpr uint16_t PULSE_PADDING = 20; // microseconds

    void write();

private:
    // use to index the devices array
    static constexpr uint8_t RIGHT_MOTOR = 0;
    static constexpr uint8_t RIGHT_ALR = 1;
    static constexpr uint8_t LEFT_MOTOR = 2;
    static constexpr uint8_t LEFT_ALR = 3;
    static constexpr uint8_t NUM_DEVICES = 4;

    struct Device
    {
        unsigned long timer;
        uint16_t pulsewidth;
        uint8_t pin;
    };

    Device devices[NUM_DEVICES];

    template<typename Type>
    void swap(Type& a, Type& b)
    {
        Type temp = a;
        a = b;
        b = temp;
    }
};


#endif // PWM_SCHEDULAR_H
