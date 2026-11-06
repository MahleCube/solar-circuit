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
    uint32_t pwm_events; /* retained PIO IRQ bits: stop=1, PV cause=2, lease=4 */
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
    C07_FAULT, C07_WAIT_INPUT, C07_CHARGE_HOLD } c07_state;

typedef struct {
    c07_state state;
    const char *fault;
    uint64_t entered_us, previous_us, stable_since_us, rearm_started_us;
    bool previous_valid, stable_valid, rearm_started, rearm_finished;
    bool reset_was_requested, stop_battery_enabled;
    c07_real_t open_voltage;
    /* Feedback-response diagnostic, independent of the current PI integrator.
     * Uses confirmed previous PWM and independent input-current observations;
     * it does not equate one phase sample of IL with its cycle average. */
    bool feedback_window_valid;
    uint32_t feedback_window_us;
    c07_real_t feedback_start_vin, feedback_prev_vin, feedback_prev_vout;
    c07_real_t feedback_prev_ipv, feedback_prev_duty;
    c07_real_t feedback_input_charge, feedback_dcm_charge, feedback_volt_seconds;
    c07_real_t feedback_max_ripple;
    c07_real_t feedback_ripple_per_v_duty; /* immutable 1/(fsw*Lmin), init-owned */
    c07_real_t feedback_diode_loss_v; /* immutable Vf + modeled Buck loss */
    /* Independent nonzero-current input-charge window. Never mix pre-fault
     * large IL into the existing near-zero response/drive diagnostic above. */
    bool input_feedback_window_valid;
    uint32_t input_feedback_window_us;
    c07_real_t input_feedback_start_vin, input_feedback_prev_vin;
    c07_real_t input_feedback_prev_vout, input_feedback_prev_ipv;
    c07_real_t input_feedback_prev_il, input_feedback_prev_duty;
    c07_real_t input_feedback_charge, input_feedback_upper_charge;
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
