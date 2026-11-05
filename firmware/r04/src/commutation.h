#ifndef C07_COMMUTATION_H
#define C07_COMMUTATION_H
#include <stdbool.h>
#include <stdint.h>

/* These are selected design-policy limits, NOT instantaneous hardware
 * protection ratings or evidence of physical commissioning. Their application
 * depends on sensor RC/ADC timing, packet gaps, faults and both diode paths.
 * A sampled current threshold cannot certify the intervening current peak. */
typedef struct {
    bool enabled; /* Select this configuration; does not certify the hardware. */
    double hs_off_max_s, hs_on_min_s, hs_on_max_s;
    double ls_off_max_s, ls_on_min_s, ls_on_max_s, nonoverlap_margin_s;
    double current_error_a;
    double vin_error_v, vout_error_v;
    double minimum_current_a, maximum_current_a;
    double sync_enter_a, sync_exit_a;
    uint32_t sync_entry_dwell_us;
    double input_headroom_v;
} c07_commutation_config;
typedef struct { bool synchronous, entry_pending, clock_valid; uint64_t entry_since_us,last_us; } c07_commutation;
typedef struct {
    double vin, vout, il, duty;
    bool charging;
    uint64_t now_us;
} c07_commutation_input;
typedef struct { unsigned high_ticks,low_ticks; bool synchronous; } c07_gate_plan;
const char *c07_commutation_validate(const c07_commutation_config *);
void c07_commutation_reset(c07_commutation *);
const char *c07_commutation_plan(c07_commutation *, const c07_commutation_config *,
                                const c07_commutation_input *, c07_gate_plan *);
#endif
