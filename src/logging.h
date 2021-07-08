#ifndef __LOGGING_H__
#define __LOGGING_H__

#include <stdio.h>

#include "pico/stdlib.h"

#include "flight_controller.h"

#define PRINT_INPUTS
#define PRINT_ORIENTATION
//#define PRINT_TARGETS
//#define PRINT_PIDS
#define PRINT_OUTPUTS
#define PRINT_CTRL_MODE
#define PRINT_FLIGHT_MODE
#define PRINT_TSTATE
#define PRINT_FLAGS

#ifdef DEBUG
#   define PRINTF_DEBUG(f, ...) \
        printf(f, ##__VA_ARGS__)
#else
#   define PRINTF_DEBUG(f, ...)
#endif

void do_logging(void);
void dump_logs(void);

#if defined(PRINT_INPUTS) || \
    defined(PRINT_ORIENTATION) || \
    defined(PRINT_TARGETS) || \
    defined(PRINT_PIDS) || \
    defined(PRINT_OUTPUTS) || \
    defined(PRINT_CTRL_MODE) || \
    defined(PRINT_FLIGHT_MODE) || \
    defined(PRINT_TSTATE) || \
    defined(PRINT_FLAGS)
#       define DO_USB_LOGGING
#endif

#endif // __LOGGING_H__
