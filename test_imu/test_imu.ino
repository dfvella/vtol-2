#include <Arduino.h>

#include "imu_driver.h"

Imu imu {LED_BUILTIN};

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(9600);
    Serial.println("Calibrating imu...");

    imu.calibrate();
}

void loop()
{
    static unsigned long timer = micros();

    imu.run();

    Serial.print(imu.roll());
    Serial.print(' ');
    Serial.print(imu.pitch());
    Serial.print(' ');
    Serial.println(imu.yaw());

    while (micros() - timer < 20000);
    timer = micros();
}
