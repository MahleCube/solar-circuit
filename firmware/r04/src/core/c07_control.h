#ifndef C07_CONTROL_H
#define C07_CONTROL_H

/* Portable c07 regulation core. No GPIO, heap, clocks, fault latch, or charger
 * startup sequencing. Call only with a fresh validated ADC frame after the
 * board supervisor has authorized either output ramp or charging. */
#include <stdbool.h>
#include <stdint.h>
#include <stdatomic.h>

#ifndef C07_REAL_TYPE
#define C07_REAL_TYPE float
#endif
typedef C07_REAL_TYPE c07_real_t;

#define C07_CONFIG_REAL_FIELDS(X) \
 X(DUTY_MIN, .03) X(DUTY_MAX, .90) \
 X(CHARGE_CURRENT_A, 15.) X(CHARGE_VOLTAGE_V, 57.1) \
 X(VOUT_TRIP_V, 58.) X(PV_HEADROOM_V, 5.) X(CONNECT_OFFSET_V, .1) \
 X(RAMP_CURRENT_A, .5) X(CURRENT_DEADBAND_A, .05) X(VOLTAGE_DEADBAND_V, .03) \
 X(DUTY_RISE_PER_S, .3) X(CURRENT_REDUCTION_PER_A_S, 3.) \
 X(VOLTAGE_REDUCTION_PER_V_S, 4.) X(POWER_DEADBAND_W, .2) \
 X(MPPT_INITIAL_FRACTION, .82) X(PV_KP_A_PER_V, .9) X(PV_KI_A_PER_V_S, 120.) \
 X(IL_KP_DUTY_PER_A, .002) X(IL_KI_DUTY_PER_A_S, 1.) \
 X(CV_KP_A_PER_V, 4.) X(CV_KI_A_PER_V_S, 5.) \
 X(IL_RISE_A_PER_S, 0.) X(ZERO_CURRENT_TARGET_A, .001) \
 X(PV_KD_A_S_PER_V, .005) X(PV_REFERENCE_WEIGHT, .5) \
 X(INPUT_CURRENT_LIMIT_A, 13.5) X(INPUT_LIMIT_REDUCTION_A_PER_S, 200.) \
 X(INPUT_FOLDBACK_BAND_V, 2.) X(BUCK_LOSS_MARGIN_V, .5) \
 X(TSINK_DERATE_C, 65.) X(TSINK_MAX_C, 80.) \
 X(MPPT_SETTLE_V, .3) X(MPPT_SETTLE_A, .3) \
 X(MPPT_STEP_MIN_V, .05) X(MPPT_STEP_MAX_V, .5) X(MPPT_STEP_GAIN_V, 5.) \
 X(MPPT_POWER_FLOOR_W, 10.) X(MPPT_VOC_MARGIN_V, .5) \
 X(MPPT_REFERENCE_MAX_V, 80.) X(MPPT_NOLOAD_A, .1) \
 X(MPPT_VOLTAGE_DEADBAND_V, .02835360440340909) \
 X(MPPT_CURRENT_DEADBAND_A, .01220703125) \
 X(MPPT_SLOPE_CURRENT_FLOOR_A, .1)

#define C07_CONFIG_TIME_FIELDS(X) \
 X(CONTROL_PERIOD_US, 1001) X(MAX_LOOP_GAP_US, 5000) \
 X(VOLTAGE_PERIOD_US, 1001) X(MPPT_PERIOD_US, 100000) \
 X(IL_FILTER_US, 0) X(VOLTAGE_FILTER_US, 1000) X(PV_D_FILTER_US, 3000) \
 X(MPPT_SETTLE_US, 20000) X(MPPT_WINDOW_US, 20000) \
 X(MPPT_REACQUIRE_US, 200000) X(MPPT_MAX_PAIR_SPAN_US, 300)

typedef struct {
    unsigned MPPT_METHOD; /* 0: exact legacy path, 1: P&O, 2: IncCond */
    unsigned MPPT_STEP_MODE; /* 0: fixed, 1: relative power, 2: normalized secant slope */
#define C07_REAL_FIELD(name, value) c07_real_t name;
    C07_CONFIG_REAL_FIELDS(C07_REAL_FIELD)
#undef C07_REAL_FIELD
#define C07_TIME_FIELD(name, value) uint32_t name;
    C07_CONFIG_TIME_FIELDS(C07_TIME_FIELD)
#undef C07_TIME_FIELD
} c07_config;

typedef struct {
    c07_real_t vin, ipv, vout, il, tbat, tsink, vbat, vpv;
    bool valid;
    int32_t pv_pair_span_us; /* actual software transfer bounds, not ADC aperture */
} c07_sample;

typedef enum {
    C07_LIMIT_STOPPED, C07_LIMIT_STARTUP, C07_LIMIT_MPPT, C07_LIMIT_CV,
    C07_LIMIT_VOLTAGE, C07_LIMIT_INPUT_CURRENT, C07_LIMIT_INPUT_FOLDBACK,
    C07_LIMIT_BUCK_UNREACHABLE, C07_LIMIT_THERMAL, C07_LIMIT_CURRENT,
    C07_LIMIT_MIN_DUTY, C07_LIMIT_MAX_DUTY
} c07_limit;

typedef enum {
    C07_OK=0, C07_ERR_CONFIG=1, C07_ERR_DT=2,
    C07_ERR_SAMPLE=3, C07_ERR_NUMERIC=4, C07_ERR_ARGUMENT=5
} c07_status;

typedef struct { c07_real_t kp, ki, integral; } c07_pi;
typedef struct {
    c07_real_t maximum, target, step_v, previous_power, power_sum, voltage_sum;
    c07_real_t current_sum, previous_voltage, previous_current;
    c07_real_t last_mean_voltage, last_mean_current, last_mean_power, last_mean_instant_power, last_signal;
    uint32_t elapsed_us, settled_us, window_us, updates;
    uint32_t moves, holds;
    int direction;
    bool have_previous_power;
} c07_mppt;

/* Single-core IRQ/main mailbox. Each payload is published by an aligned
 * 32-bit release store; fast IRQ never retries a payload copy. */
typedef struct {
    uint64_t epoch, captured_us;
    uint32_t request_id;
    c07_real_t minimum;
    c07_mppt state;
} c07_mppt_job;
typedef struct {
    c07_mppt_job job;
    bool valid;
} c07_mppt_result;
typedef struct {
    _Atomic uint32_t request_seq, result_seq;
    volatile c07_mppt_job request;
    volatile c07_mppt_result result;
    uint32_t background_seen; /* main writer only */
    uint64_t epoch, clock_us, pending_since_us; /* fast writer only */
    uint32_t next_request, pending_request, consumed_result;
    uint32_t published, applied, rejected, expired, invalidated;
    bool pending;
} c07_mppt_mailbox;

typedef struct {
    c07_config cfg;
    c07_pi pv_pi, current_pi, cv_pi;
    c07_mppt mppt;
    c07_mppt_mailbox async;
    c07_real_t duty, current_target, current, output_voltage, pv_voltage, power;
    c07_real_t cv_ceiling, pv_previous, pv_derivative;
    uint32_t outer_elapsed_us, reacquire_elapsed_us, reacquisitions;
    bool initialized, tracking_started, limited, restart_feedforward_pending;
    bool off_current_hold;
    c07_limit limit_reason;
} c07_controller;

void c07_config_defaults(c07_config *cfg);
int c07_init(c07_controller *controller, const c07_config *cfg);
void c07_reset(c07_controller *controller);
int c07_start_tracking(c07_controller *controller, c07_real_t open_voltage,
                       c07_real_t battery_voltage);
/* charging=false: original limited COUT ramp; charging=true: cascaded PI+MPPT.
 * Returns error with duty_out=0 and controller reset on invalid input. The
 * caller must immediately disable PWM and latch the supervisory fault. */
int c07_step(c07_controller *controller, const c07_sample *sample,
             uint32_t dt_us, bool charging, c07_real_t *duty_out);
/* c07_step never executes background work. begin copies a stable snapshot,
 * compute uses only local state and immutable config, publish commits the
 * result payload. These calls are main-owned and may be interrupted by step. */
bool c07_mppt_begin(c07_controller *, c07_mppt_job *);
bool c07_mppt_compute(const c07_config *, const c07_mppt_job *, c07_mppt_result *);
void c07_mppt_publish(c07_controller *, const c07_mppt_result *);
bool c07_mppt_background(c07_controller *);
void c07_mppt_invalidate(c07_controller *, bool clear_anchor);
const char *c07_limit_name(c07_limit reason);
#endif
