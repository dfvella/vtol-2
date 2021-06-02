#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

#define MICROSEC_PER_SEC 1000000.0

#define PID_INITIAL_OUTPUT 1500
#define PID_MAX_OUTPUT 500

#define FIR_BUFFER_SIZE 10

enum class Flight_Mode : uint8_t { TO_FORWARD, FORWARD, TO_SLOW, SLOW, TO_VERTICAL, VERTICAL };

class FIR_Filter {
public:
    using Response = float[FIR_BUFFER_SIZE];

    FIR_Filter(float* response_in);
    FIR_Filter(const FIR_Filter& other);
    FIR_Filter& operator=(const FIR_Filter& other);
    ~FIR_Filter();

    float calculate(float input);
    void flush();

private:
    const float* response;

    float* buffer;
    size_t front;

    uint8_t startup_counter;
};

class PIDcontroller
{
public:
    struct Gains
    {
        float p;
        float i;
        float d;
        float i_max;
    };

    PIDcontroller(const Gains& forward_in, const Gains& slow_in, 
                const Gains& vertical_in, const Flight_Mode& mode_in);
    float calculate(float error);

    float get_error();

private:
    const Gains& select_gains();

    const Gains& forward_gains;
    const Gains& slow_gains;
    const Gains& vertical_gains;

    const Flight_Mode& flight_mode;

    float i_output, prev_output, prev_error;

    unsigned long timer;

    FIR_Filter::Response d_response = {
        //1, 0, 0, 0, 0, 0, 0, 0, 0, 0 // no filter
        0.4, 0.3, 0.2, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 // delay <20 ms
        //0.3, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 // delay ~25 ms
    };

    FIR_Filter d_filter;
};

#endif // PID_CONTROLLER_H
