#include <Arduino.h>

#include "radio_driver.h"
#include "pwm_schedular.h"
#include "flight_controller.h"

#include "serial_logger.h"

#define ENABLE_SERVOS

#define RADIO_PIN 3

unsigned long timer = 0;
const int LOOP_TIME = 5000; // microseconds

Radio radio(RADIO_PIN);

PWMSchedular pwm;

Flight_Controller flight_controller;

enum class Controller_State : uint8_t { PPMSYNC, PIDCALC, SERVOSET, SERVOWRITE };

void setup() 
{
    pinMode(LED_BUILTIN, OUTPUT);

    #ifdef SERIAL_CONNECTION
    Serial.begin(SERIAL_BAUD_RATE);
    Serial.println("Serial connection");
    #endif

    flight_controller.begin();

    radio.begin();

    pwm.begin();

    digitalWrite(LED_BUILTIN, LOW);

    timer = micros();

    #ifdef SERIAL_CONNECTION
    Serial.println("setup complete");
    #endif
}

void loop() 
{
    static Controller_State state = Controller_State::PPMSYNC;

    static Flight_Controller::Input controller_input;
    static Flight_Controller::Output controller_output;

    flight_controller.run();

    switch (state)
    {
    case Controller_State::PPMSYNC:
        controller_input = {
            radio.thr(),
            radio.arl(),
            radio.ele(),
            radio.rud(),
            radio.ger(),
            radio.aux()
        };

        state = Controller_State::PIDCALC;
        break;

    case Controller_State::PIDCALC:
        flight_controller.calculate_outputs(controller_input, controller_output);

        state = Controller_State::SERVOSET;
        break;

    case Controller_State::SERVOSET:
        pwm.set_right_motor(controller_output.right_motor);
        pwm.set_right_tilt(controller_output.right_tilt);
        pwm.set_right_aileron(controller_output.right_alr);
        pwm.set_left_motor(controller_output.left_motor);
        pwm.set_left_tilt(controller_output.left_tilt);
        pwm.set_left_aileron(controller_output.left_alr);
        pwm.set_elevator(controller_output.elevator);

        #ifdef DO_LOGGING_50HZ
        print_log()
        #endif

        state = Controller_State::SERVOWRITE;
        break;

    case Controller_State::SERVOWRITE:
        #ifdef ENABLE_SERVOS
        pwm.write();
        #endif 

        state = Controller_State::PPMSYNC;
        break;
    }

    #ifdef DO_LOGGING_200HZ
    print_log()
    #endif

    if (micros() - timer > LOOP_TIME) 
        digitalWrite(LED_BUILTIN, HIGH);
    else
        digitalWrite(LED_BUILTIN, LOW);

    while (micros() - timer < LOOP_TIME);
    timer = micros();
}
