#include "reboot.h"

// Reboot and start executing program in flash.
// Do this by enabling the watchdog and letting the timer expire.
void reboot(void) {
    watchdog_enable(1, 0);
    for (;;) {
        tight_loop_contents();
    }
}

// Reboot and put the board into bootsel mode.
void bootsel(void) {
    reset_usb_boot(0, 0);
}
