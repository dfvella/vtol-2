#include "logging.h"

static uint8_t buffer[FLASH_PAGE_SIZE];

static void print_state(const Fc_State* state) {
#   ifdef PRINT_INPUTS
        printf("%f %f %f %f %f %f ",
            state->input.thro,
            state->input.aile,
            state->input.elev,
            state->input.rudd,
            state->input.gear,
            state->input.aux1
        );
#   endif // PRINT_INPUTS
#   ifdef PRINT_ORIENTATION
        printf("%f %f %f ",
            state->roll,
            state->pitch,
            state->yaw
        );
#   endif // PRINT_ORIENTATION
#   ifdef PRINT_TARGETS
        printf("%f %f %f ",
            state->target_roll,
            state->target_pitch,
            state->target_yaw
        );
#   endif // PRINT_TARGETS
#   ifdef PRINT_PIDS
        printf("%f %f %f ",
            state->pid_output_roll,
            state->pid_output_pitch,
            state->pid_output_yaw
        );
#   endif // PRINT_PIDS
#   ifdef PRINT_OUTPUTS
        printf("%f %f %f %f ",
            state->output.right_elevon,
            state->output.left_elevon,
            state->output.right_motor,
            state->output.left_motor
        );
#   endif // PRINT_OUTPUTS
#   ifdef PRINT_CTRL_MODE
        printf("%d ", state->ctrl_mode);
#   endif // PRINT_CTRL_MODE
#   ifdef PRINT_FLIGHT_MODE
        printf("%d ", state->flight_mode);
#   endif // PRINT_FLIGHT_MODE
#   ifdef PRINT_TSTATE
        printf("%d ", state->tstate);
#   endif // PRINT_TSTATE
#   ifdef PRINT_FLAGS
        printf("%d ", state->flags);
#   endif // PRINT_FLAGS
    printf("\n");
}

void init_logging(void) {
    // clear flash contents
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(LOG_FLASH_START, LOG_FLASH_SIZE_BYTES);
    restore_interrupts(ints);

#   if LOG_FLASH_START % FLASH_SECTOR_SIZE
#       error "start of flash log not aligned with flash sector"
#   endif

#   if LOG_FLASH_SIZE_BYTES % FLASH_SECTOR_SIZE
#       error "flash log size is not a multiple of flash sector size"
#   endif
}

void do_logging(void) {
    static uint32_t flash_offset = LOG_FLASH_START;
    static uint8_t loop_counter = 0;

    const Fc_State *state = fc_get_state();

#   ifdef DO_USB_LOGGING
        print_state(state);
#   endif

    Log_Data log_data;

    log_data.input_thro = (int8_t) state->input.thro;
    log_data.input_aile = (int8_t) state->input.aile;
    log_data.input_elev = (int8_t) state->input.elev;
    log_data.input_rudd = (int8_t) state->input.rudd;
    log_data.input_gear = (int8_t) state->input.gear;
    log_data.input_aux1 = (int8_t) state->input.aux1;

    log_data.current_roll = state->roll;
    log_data.current_pitch = state->pitch;
    log_data.current_yaw = state->yaw;

    log_data.target_roll = state->target_roll;
    log_data.target_pitch = state->target_pitch;
    log_data.target_yaw = state->target_yaw;

    log_data.pid_roll = state->pid_output_roll;
    log_data.pid_pitch = state->pid_output_pitch;
    log_data.pid_yaw = state->pid_output_yaw;

    log_data.output_right_elevon = state->output.right_elevon;
    log_data.output_left_elevon = state->output.left_elevon;
    log_data.output_right_motor = state->output.right_motor;
    log_data.output_left_motor = state->output.left_motor;
    log_data.output_gear = (int8_t) state->output.gear;

    log_data.ctrl_mode = (uint8_t) state->ctrl_mode;
    log_data.flight_mode = (uint8_t) state->flight_mode;
    log_data.tstate = (uint8_t) state->tstate;
    log_data.flags = (uint8_t) state->flags;

    uint8_t offset = loop_counter * sizeof(Log_Data);
    memcpy(buffer + offset, &log_data, sizeof(Log_Data));

    uint32_t ints = save_and_disable_interrupts();
    flash_range_program(flash_offset, buffer, FLASH_PAGE_SIZE);
    restore_interrupts(ints);

    if (loop_counter == 3) {
        flash_offset += FLASH_PAGE_SIZE;
        if (flash_offset >= PICO_FLASH_SIZE_BYTES) {
            flash_offset = LOG_FLASH_START;
        }
    }

    loop_counter = (loop_counter + 1) % 4;
}

void dump_logs(void) {
    const Log_Data *flash = (const Log_Data *)(XIP_BASE + LOG_FLASH_START);

    while ((uint32_t)(flash) - XIP_BASE < PICO_FLASH_SIZE_BYTES) {
        printf("%d, %d, %d, %d, %d, %d, ",
            flash->input_thro,
            flash->input_aile,
            flash->input_elev,
            flash->input_rudd,
            flash->input_gear,
            flash->input_aux1
        );
        printf("%f, %f, %f, ",
            flash->current_roll,
            flash->current_pitch,
            flash->current_yaw
        );
        printf("%f, %f, %f, ",
            flash->target_roll,
            flash->target_pitch,
            flash->target_yaw
        );
        printf("%f, %f, %f, ",
            flash->pid_roll,
            flash->pid_pitch,
            flash->pid_yaw
        );
        printf("%f, %f, %f, %f, %d, ",
            flash->output_right_elevon,
            flash->output_left_elevon,
            flash->output_right_motor,
            flash->output_left_motor,
            flash->output_gear
        );
        printf("%d, %d, %d, %d\n",
            flash->ctrl_mode,
            flash->flight_mode,
            flash->tstate,
            flash->flags
        );

        ++flash;
    }
}
