#ifndef CONSTANTS_H
#define CONSTANTS_H

#define RIGHT_ELEVON_PIN 12
#define LEFT_ELEVON_PIN 13
#define RIGHT_MOTOR_PIN 14
#define LEFT_MOTOR_PIN 15

#define STATUS_LED_PIN 25
#define I2C_BAUD_RATE_HZ 400 * 1000

#define I2C_SDA_PIN 20
#define I2C_SCL_PIN 21

#define AR610_THRO_PIN 1
#define AR610_AILE_PIN 3
#define AR610_ELEV_PIN 5
#define AR610_RUDD_PIN 7
#define AR610_GEAR_PIN 9
#define AR610_AUX1_PIN 11

#define STDIO_WAIT 2 // units: seconds
#define USB_WAIT 3 // units: seconds

#define LOOP_PERIOD_US 4000
#define USB_TIMEOUT_PADDING_US 500

#define COMMAND_DUMP_LOGS 'd'
#define COMMAND_REBOOT 'r'
#define COMMAND_BOOTSEL 'b'

#endif // CONSTANTS_H
