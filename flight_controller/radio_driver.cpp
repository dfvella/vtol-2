#include "radio_driver.h"

volatile uint16_t Radio::current_buffer[BUFFER_SIZE];
volatile uint16_t* Radio::chan_ptr = Radio::current_buffer;
volatile uint16_t* Radio::sync_ptr = Radio::current_buffer;
volatile unsigned long Radio::cppm_timer = 0;

Radio::Radio(uint8_t pin_in) :
    pin{ pin_in }
{ }

void Radio::begin()
{
    pinMode(pin, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(pin), toggle, CHANGE);

    #ifdef APPLY_PPM_RAMP_FILTER

    for (int i = 0; i < BUFFER_SIZE; ++i)
    {
        previous_buffer[i] = MID_PPM_PULSEWIDTH;
        filter_timer[i] = micros();
    }

    #endif // APPLY_PPM_RAMP_FILTER

    sync();
}

// Checks if the sync_ptr is pointed to the sync channel in the 
// data array. If not, the data array is traversed until the 
// sync channel is found. MAXIMUM_RESYNC_ATTEMPTS limits the 
// number of complete array traverses.
void Radio::sync() 
{
    for (uint8_t i = 0; i < MAXIMUM_RESYNC_ATTEMPTS; ++i) 
    {
        if (*sync_ptr > MIN_SYNC_PULSEWIDTH) 
            break;

        ++sync_ptr;

        if (sync_ptr >= current_buffer + BUFFER_SIZE)
            sync_ptr = current_buffer;
    }
}

// Returns the pulsewidth of the corresponding receiver channel 
// in milliseconds.
uint16_t Radio::get(uint8_t index) 
{
    sync();

    int val;

    if (sync_ptr + index < current_buffer + BUFFER_SIZE)
        val = sync_ptr[index];
    else
        val = sync_ptr[index - BUFFER_SIZE];

    #ifdef APPLY_PPM_RAMP_FILTER

    uint32_t max_delta = PPM_MAX_DELTA_20MS * ((micros() - filter_timer[index]) / 20000.0f);
    filter_timer[index] = micros();

    val = constrain(val, previous_buffer[index] - max_delta, previous_buffer[index] + max_delta);
    previous_buffer[index] = val;

    #endif // APPLY_PPM_RAMP_FILTER

    val = constrain(val, MIN_PPM_PULSEWIDTH, MAX_PPM_PULSEWIDTH);

    #ifdef CONVERT_OUTPUT_STD_PWM
    val += PPM_PWM_OFFSET;
    #endif

    return val;
}

uint16_t Radio::arl()
{
    return get(INDEX_ARL);
}

uint16_t Radio::ele()
{
    return get(INDEX_ELE);
}

uint16_t Radio::thr()
{
    return get(INDEX_THR);
}

uint16_t Radio::rud()
{
    return get(INDEX_RUD);
}

uint16_t Radio::ger()
{
    return get(INDEX_GER);
}

uint16_t Radio::aux()
{
    return get(INDEX_AUX);
}

// Updates the data array. Called by interrupt routine
void Radio::toggle()
{
    *chan_ptr = micros() - cppm_timer;
    cppm_timer = micros();

    ++chan_ptr;

    if (chan_ptr >= current_buffer + BUFFER_SIZE)
        chan_ptr = current_buffer;
}
