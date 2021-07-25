#ifndef __REBOOT_H__
#define __REBOOT_H__

#include "pico/bootrom.h"
#include "hardware/watchdog.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

void reboot(void);
void bootsel(void);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // __REBOOT_H__
