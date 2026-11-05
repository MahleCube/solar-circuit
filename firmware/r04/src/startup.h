#ifndef C07_STARTUP_H
#define C07_STARTUP_H
#include <stdbool.h>
#include <stdint.h>
#include "c07_control.h"

/* No Pico SDK types: this supervisor also runs in host tests. */
typedef struct {
    c07_sample sample;
    uint64_t sample_us, completed_us, frame_id;
    uint32_t age_us, adc_start_us[8], adc_end_us[8];
    bool bias_good, battery_fault, pv_fault, pv_stopped;
    const char *error;
} c07_inputs;

typedef struct { bool run, reset_fault; } c07_request;
typedef struct {
    uint64_t sample_us;
    bool bias_enable, pv_enable, battery_enable, pv_rearm, synchronous_allowed;
    c07_real_t duty;
} c07_command;

typedef enum { C07_IDLE, C07_CLEAR, C07_PV_CONNECT, C07_BIAS,
    C07_RAMP_OUTPUT, C07_CONNECT_BATTERY, C07_CHARGING, C07_STOPPING,
    C07_FAULT, C07_WAIT_INPUT } c07_state;

typedef struct {
    c07_state state;
    const char *fault;
    uint64_t entered_us, previous_us, stable_since_us, rearm_started_us;
    bool previous_valid, stable_valid, rearm_started, rearm_finished;
    bool reset_was_requested, stop_battery_enabled;
    c07_real_t open_voltage;
    c07_controller regulator;
    c07_command command;
} c07_charger;

int c07_charger_init(c07_charger *, const c07_config *);
void c07_charger_trip(c07_charger *, const char *, uint64_t);
void c07_charger_wait_input(c07_charger *, uint64_t);
const c07_command *c07_charger_step(c07_charger *, const c07_inputs *,
                                    const c07_request *, uint64_t);
const char *c07_state_name(c07_state);
const char *c07_measurement_fault(const c07_inputs *);
#endif
