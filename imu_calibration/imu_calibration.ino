#include <Arduino.h>

#include "imu_driver.h"

Imu imu{ LED_BUILTIN };

void setup()
{
    Serial.begin(9600);
    pinMode(LED_BUILTIN, OUTPUT);
    imu.calibrate_accel();
}

void loop() { }
