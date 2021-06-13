#include <mpu6050.h>
#include <ar610.h>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "pwm.h"

#ifdef DEBUG
#   include <stdio.h>
#endif

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

enum fc_flags {
    IMU_CONNECTED = 1,
    RADIO_CONNECTED = 2,
    BMP_CONNECTED = 4
};

void init_all(i2c_inst_t* i2c, mpu6050_inst_t* mpu, ar610_inst_t* ar) {

}

int main() {
    // Initialize usb serial port if DEBUG build
#   ifdef DEBUG
    stdio_init_all();
    sleep_ms(3000);
    printf("Starting flight controller in debug mode...\n");
#   endif

    // Initialize status led
    gpio_init(STATUS_LED_PIN);
    gpio_set_dir(STATUS_LED_PIN, GPIO_OUT);

    // Initialize I2C bus
    i2c_init(&i2c0_inst, I2C_BAUD_RATE_HZ);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // Initialize IMU
#   ifdef DEBUG
    printf("Initializing IMU...\n");
#   endif

    mpu6050_inst_t mpu6050;
    uint8_t imu_error = mpu6050_init(&mpu6050, i2c_default, STATUS_LED_PIN);

#   ifdef DEBUG
    if (imu_error) {
        printf("mpu6050 not found\n");
        return 1;
    }
#   endif

    // Initialize radio receiver
#   ifdef DEBUG
    printf("Initializing Radio...\n");
#   endif

    ar610_inst_t ar610;
    ar610_init(&ar610,
        AR610_THRO_PIN,
        AR610_AILE_PIN,
        AR610_ELEV_PIN,
        AR610_RUDD_PIN,
        AR610_GEAR_PIN,
        AR610_AUX1_PIN
    );

    // Initialize pwm outputs
#   ifdef DEBUG
    printf("Initializing PWM outputs...\n");
#   endif

    pwm_init_all_outputs();

    absolute_time_t time = get_absolute_time();
    while (1) {
        ar610_update_state(&ar610);

        uint16_t pulse = ar610_get_aile(&ar610);
        printf("%d\n",pulse);

        if (ar610_is_connected(&ar610)) {
            pwm_set_right_elevon(pulse);
        } else {
            printf("ar610 disconnected\n");
        }

        while (absolute_time_diff_us(time, get_absolute_time()) < 20000);
        time = get_absolute_time();
    }

    return 0;
}
