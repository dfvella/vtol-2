#include <Arduino.h>

#include "radio_driver.h"

#define RADIO_PIN 3
#define LOOP_FREQ 50 // Hz

Radio radio(RADIO_PIN);
unsigned long timer = 0;

void setup()
{
    Serial.begin(9600);
    radio.begin();
}

void loop()
{
    // Serial.print(radio.arl());
    // Serial.print(' ');
    // Serial.print(radio.ele());
    // Serial.print(' ');
    // Serial.print(radio.rud());
    // Serial.print(' ');
    // Serial.print(radio.thr());
    // Serial.print(' ');
    Serial.print(radio.ger());
    Serial.print(' ');
    Serial.print(radio.aux());
    Serial.println();

    //Serial.println(micros() - timer);

    while (micros() - timer < (1.0f / LOOP_FREQ) * 1000000);
    timer = micros();
}
