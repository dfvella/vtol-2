#include "logging.h"

void do_logging(void) {
    PRINTF_DEBUG("Starting logger\n");

    // get a pointer to the flight controller state object
    const Fc_State *state = fc_get_state();

    for (;;) {
#       ifdef DO_USB_LOGGING
#           ifdef PRINT_INPUTS
                printf("%f %f %f %f %f %f ",
                    state->input.thro,
                    state->input.aile,
                    state->input.elev,
                    state->input.rudd,
                    state->input.gear,
                    state->input.aux1
                );
#           endif // PRINT_INPUTS
#           ifdef PRINT_ORIENTATION
                printf("%f %f %f ",
                    state->roll,
                    state->pitch,
                    state->yaw
                );
#           endif // PRINT_ORIENTATION
#           ifdef PRINT_TARGETS
                printf("%f %f %f ",
                    state->target_roll,
                    state->target_pitch,
                    state->target_yaw
                );
#           endif // PRINT_TARGETS
#           ifdef PRINT_PIDS
                printf("%f %f %f ",
                    state->pid_output_roll,
                    state->pid_output_pitch,
                    state->pid_output_yaw
                );
#           endif // PRINT_PIDS
#           ifdef PRINT_OUTPUTS
                printf("%f %f %f %f ",
                    state->output.right_elevon,
                    state->output.left_elevon,
                    state->output.right_motor,
                    state->output.left_motor
                );
#           endif // PRINT_OUTPUTS
#           ifdef PRINT_CTRL_MODE
                printf("%d ", state->ctrl_mode);
#           endif // PRINT_CTRL_MODE
#           ifdef PRINT_FLIGHT_MODE
                printf("%d ", state->flight_mode);
#           endif // PRINT_FLIGHT_MODE
#           ifdef PRINT_TSTATE
                printf("%d ", state->tstate);
#           endif // PRINT_TSTATE
#           ifdef PRINT_FLAGS
                printf("%d ", state->flags);
#           endif // PRINT_FLAGS
            printf("\n");
#       endif // DO_USB_LOGGING
    }
}

void dump_logs(void) {
    return;
}
