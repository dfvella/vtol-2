// prints PWM inputs over USB serial port

#include <stdio.h>
#include <ar610.h>

#include "pico/stdlib.h"
#include "pico/time.h"

#define AR610_THRO_PIN 1
#define AR610_AILE_PIN 3
#define AR610_ELEV_PIN 5
#define AR610_RUDD_PIN 7
#define AR610_GEAR_PIN 9
#define AR610_AUX1_PIN 11

int main() {
    stdio_init_all();

    ar610_inst_t ar610;
    ar610_init(&ar610,
        AR610_THRO_PIN,
        AR610_AILE_PIN,
        AR610_ELEV_PIN,
        AR610_RUDD_PIN,
        AR610_GEAR_PIN,
        AR610_AUX1_PIN
    );

    absolute_time_t timer = get_absolute_time();
    while (1) {
        ar610_update_state(&ar610);

        if (ar610_is_connected(&ar610)) {
            printf("thro: %u aile: %u elev: %u rudd: %u gear: %u aux1: %u\n",
                ar610_get_thro(&ar610),
                ar610_get_aile(&ar610),
                ar610_get_elev(&ar610),
                ar610_get_rudd(&ar610),
                ar610_get_gear(&ar610),
                ar610_get_aux1(&ar610)
            );
        } else {
            printf("transmitter disconnected\n");
        }

        while (absolute_time_diff_us(timer, get_absolute_time()) < 20000);
        timer = get_absolute_time();
    }
}
