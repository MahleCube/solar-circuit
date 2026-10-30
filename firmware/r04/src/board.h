#ifndef C07_BOARD_H
#define C07_BOARD_H
#include "startup.h"
#include "rearm.h"
#include "hardware/pio.h"

typedef struct {
    PIO pio;
    unsigned sm, offset;
    pio_sm_config pwm_config;
    bool initialized, pwm_enabled, bias_enabled, pv_enabled, battery_enabled;
    c07_rearm rearm;
    uint64_t completed_frames, pwm_accepted, pwm_rejected;
    c07_inputs inputs;
    uint16_t raw[8];
} c07_board;

/* Zero return means accepted. Negative means all outputs were stopped. */
int c07_board_init(c07_board *);
void c07_board_stop(c07_board *);
const c07_inputs *c07_board_read(c07_board *);
const char *c07_board_apply(c07_board *, const c07_command *);
#endif
