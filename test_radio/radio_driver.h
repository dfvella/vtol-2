#ifndef RADIO_DRIVER_H
#define RADIO_DRIVER_H

#include <Arduino.h>

// Arduino drivers for the OrangeRX R617XL 6 channel DSM2/DSMX radio receiver.
// Decodes CPPM signal using interupts. Functions return raw CPPM pulsewidths or
// the equivalent standard PWM pulsewidths

//
// IMPORTANT: only one instance of this class can exist at one time
//

// channel order
#define RADIO_CHANNEL_ALR 1
#define RADIO_CHANNEL_ELE 2
#define RADIO_CHANNEL_THR 3
#define RADIO_CHANNEL_RUD 4
#define RADIO_CHANNEL_GER 5
#define RADIO_CHANNEL_AUX 6
#define RECEIVER_CHANNELS 6

// uncomment to convert output to standard pwm pulsewidth 1-2ms
#define CONVERT_OUTPUT_STD_PWM

// maximum number of buffer entries that will be checked during one call to sync/get
#define MAXIMUM_RESYNC_ATTEMPTS 15

// uncomment to apply ramp filter
#define APPLY_PPM_RAMP_FILTER
#define PPM_MAX_DELTA_20MS 100


class Radio
{
public:
    // create radio object for reading CPPM signal on pin "pin_in"
    Radio(uint8_t pin_in);

    // configures gpio pin modes, enables interrupt, and finds sync pulse
    void begin();

    // Returns the pulsewidth of the corresponding receiver channel in milliseconds
    uint16_t arl();
    uint16_t ele();
    uint16_t thr();
    uint16_t rud();
    uint16_t ger();
    uint16_t aux();

private:
    uint16_t get(uint8_t chan);

    // Checks if the sync_ptr is pointed to the sync channel in the 
    // data array. If not, the data array is traversed until the 
    // sync channel is found or the MAXIMUM_RESYNC_ATTEMPTS limit
    // is reached.
    void sync();

    // Updates the current data buffer. Called by interrupt routine.
    static void toggle();

    static constexpr uint8_t BUFFER_SIZE = 2 * (RECEIVER_CHANNELS + 1);

    // Receiver channels used to index the data buffer
    static constexpr uint8_t INDEX_ARL = 2 * RADIO_CHANNEL_ALR;
    static constexpr uint8_t INDEX_ELE = 2 * RADIO_CHANNEL_ELE;
    static constexpr uint8_t INDEX_THR = 2 * RADIO_CHANNEL_THR;
    static constexpr uint8_t INDEX_RUD = 2 * RADIO_CHANNEL_RUD;
    static constexpr uint8_t INDEX_GER = 2 * RADIO_CHANNEL_GER;
    static constexpr uint8_t INDEX_AUX = 2 * RADIO_CHANNEL_AUX;

    static constexpr uint16_t MIN_PPM_PULSEWIDTH = 700;
    static constexpr uint16_t MID_PPM_PULSEWIDTH = 1200;
    static constexpr uint16_t MAX_PPM_PULSEWIDTH = 1700;

    static constexpr uint16_t MIN_SYNC_PULSEWIDTH = 10000;

    static constexpr uint16_t PPM_PWM_OFFSET = 300;

    volatile static uint16_t current_buffer[BUFFER_SIZE];
    volatile static uint16_t* chan_ptr;
    volatile static uint16_t* sync_ptr;
    volatile static unsigned long cppm_timer;

    uint8_t pin;

    #ifdef APPLY_PPM_RAMP_FILTER
    uint16_t previous_buffer[BUFFER_SIZE];
    unsigned long filter_timer[BUFFER_SIZE];
    #endif // APPLY_PPM_RAMP_FILTER
};

#endif // RADIO_DRIVER_H
