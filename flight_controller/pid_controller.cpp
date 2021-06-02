#include "pid_controller.h"

FIR_Filter::FIR_Filter(float* response_in) :
    response{ response_in },
    buffer{ new float[FIR_BUFFER_SIZE] },
    front{ 0 },
    startup_counter{ 0 }
{ }

FIR_Filter::FIR_Filter(const FIR_Filter& other) :
    response{ other.response },
    buffer{ new float[FIR_BUFFER_SIZE] },
    front{ other.front },
    startup_counter{ 0 }
{
    for (size_t i = 0; i < FIR_BUFFER_SIZE; ++i)
    {
        buffer[i] = other.buffer[i];
    }
}

FIR_Filter& FIR_Filter::operator=(const FIR_Filter& other)
{
    response = other.response;
    front = other.front;

    for (size_t i = 0; i < FIR_BUFFER_SIZE; ++i)
    {
        buffer[i] = other.buffer[i];
    }

    return *this;
}

FIR_Filter::~FIR_Filter()
{
    delete[] buffer;
}

float FIR_Filter::calculate(float input)
{
    front = (front + 1) % FIR_BUFFER_SIZE;

    buffer[front] = input;

    if (startup_counter < FIR_BUFFER_SIZE)
    {
        ++startup_counter;
        return input;
    }

    float result = 0;

    for (size_t i = 0; i < FIR_BUFFER_SIZE; ++i)
    {
        int buffer_index = front - i;
        if (buffer_index < 0)
        {
            buffer_index += FIR_BUFFER_SIZE;
        }
        result += buffer[buffer_index] * response[i];
    }
    return result;
}

void FIR_Filter::flush()
{
    startup_counter = 0;
}

PIDcontroller::PIDcontroller(const Gains& forward_in, const Gains& slow_in,
                    const Gains& vertical_in, const Flight_Mode& mode_in) :
    forward_gains{forward_in},
    slow_gains{slow_in},
    vertical_gains{vertical_in}, 
    flight_mode{mode_in},
    d_filter{d_response}
{ }

float PIDcontroller::calculate(float error)
{
    static bool start = true;

    const Gains& gains = select_gains();

    if (start)
    {
        timer = micros();
        start = false;

        d_filter.flush();

        return PID_INITIAL_OUTPUT;
    }
    else 
    {
        uint16_t t_delta = micros() - timer;
        timer = micros();

        float output = gains.p * error;

        i_output += (t_delta / MICROSEC_PER_SEC) * error;
        i_output = constrain(i_output, (-1 * gains.i_max) / gains.i, gains.i_max / gains.i);
        output += constrain(gains.i * i_output, -1 * gains.i_max, gains.i_max);

        float d_error = d_filter.calculate((error - prev_error) / (t_delta / MICROSEC_PER_SEC));

        output += gains.d * d_error;
        prev_error = error;

        return constrain(output, -1 * PID_MAX_OUTPUT, PID_MAX_OUTPUT);
    }
}

float PIDcontroller::get_error()
{
    return prev_error;
}

const PIDcontroller::Gains& PIDcontroller::select_gains()
{
    if (flight_mode == Flight_Mode::FORWARD ||
        flight_mode == Flight_Mode::TO_SLOW ||
        flight_mode == Flight_Mode::TO_FORWARD)
    {
        return forward_gains;
    }
    else if (flight_mode == Flight_Mode::SLOW ||
        flight_mode == Flight_Mode::TO_VERTICAL)
    {
        return slow_gains;
    }
    return vertical_gains;
}
