#ifndef C07_BOARD_H
#define C07_BOARD_H
#include "startup.h"
#include "rearm.h"
#include "pwm_engine.h"
#include "commutation.h"

typedef struct {
    c07_pwm_engine pwm;
    c07_gate_plan last_plan;
    c07_commutation commutation;
    const char *configuration_error; /* immutable profile checked at init */
    bool initialized, pwm_enabled, bias_enabled, pv_enabled, battery_enabled;
    c07_rearm rearm;
    uint64_t completed_frames, pwm_accepted, pwm_rejected;
    c07_inputs inputs; /* pwm_events retains stop causes throughout this frame */
    uint16_t raw[8];
} c07_board;

/* Zero return means accepted. Negative means all outputs were stopped. */
int c07_board_init(c07_board *);
void c07_board_stop(c07_board *);
const c07_inputs *c07_board_read(c07_board *);
const char *c07_board_apply(c07_board *, const c07_command *);
#endif
