#ifndef __LOGGING_H__
#define __LOGGING_H__

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"

#include "flight_controller.h"

#define PRINT_INPUTS
#define PRINT_ORIENTATION
#define PRINT_TARGETS
#define PRINT_PIDS
#define PRINT_OUTPUTS
#define PRINT_CTRL_MODE
#define PRINT_FLIGHT_MODE
#define PRINT_TSTATE
#define PRINT_FLAGS

// must be greater than the program size
#define LOG_FLASH_START 128 * 1024 // units: bytes
#define LOG_FLASH_SIZE_BYTES (PICO_FLASH_SIZE_BYTES - LOG_FLASH_START)

#ifdef DEBUG
#   define PRINTF_DEBUG(f, ...) \
        printf(f, ##__VA_ARGS__); \
        stdio_flush()
#else
#   define PRINTF_DEBUG(f, ...)
#endif

typedef struct {
    float current_roll;
    float current_pitch;
    float current_yaw;

    float target_roll;
    float target_pitch;
    float target_yaw;

    float pid_roll;
    float pid_pitch;
    float pid_yaw;

    float output_right_elevon;
    float output_left_elevon;
    float output_right_motor;
    float output_left_motor;
    int8_t output_gear;

    int8_t input_thro;
    int8_t input_aile;
    int8_t input_elev;
    int8_t input_rudd;
    int8_t input_gear;
    int8_t input_aux1;

    uint8_t ctrl_mode;
    uint8_t flight_mode;
    uint8_t tstate;
    uint8_t flags;
} Log_Data;

void init_logging(void);
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
