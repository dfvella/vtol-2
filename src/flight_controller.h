#ifndef __FLIGHT_CONTROLLER_H__
#define __FLIGHT_CONTROLLER_H__

#include "pico/stdlib.h"

enum Flight_Controller_Control_Mode { MANUAL = 0, RATE = 1, ANGLE = 2 };
enum Flight_Controller_Flight_Mode { DISABLED = 0, VERTICAL = 1, HORIZONTAL = 2 };

struct Flight_Controller_Input {
    uint16_t roll;
    uint16_t pitch;
    uint16_t throttle;
    uint16_t yaw;
    uint16_t gear;
    uint16_t aux;

    float roll;
    float pitch;
    float yaw;
    float altitude;
};

struct Flight_Controller_Output {
    uint16_t right_motor;
    uint16_t right_aileron;
    uint16_t left_motor;
    uint16_t left_aileron;
};

struct Flight_Controller_State {

};

typedef Flight_Controller_State Flight_Controller_State;
typedef Flight_Controller_Input Flight_Controller_Input;
typedef Flight_Controller_Output Flight_Controller_Output;

Flight_Controller_Output flight_controller_calculate(const Flight_Controller_Input* input);

#endif // __FLIGHT_CONTROLLER_H__