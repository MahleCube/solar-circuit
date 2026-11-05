#ifndef C07_COMMISSIONING_H
#define C07_COMMISSIONING_H
#include "commutation.h"

/* Portfolio operating configuration: executable under the stated design
 * assumptions, not a certificate of measured C07 hardware limits.
 * enabled selects the configuration; numerical checks still reject invalid
 * timing/limits. ARM starts the normal supervisor sequence, DISARM stops it. */
typedef struct {
    c07_commutation_config commutation;
} c07_commissioning_profile;

/* Ideal calibrated-channel model. Real per-channel gain/offset and residual
 * errors must be established separately; unity is not a calibration result. */
#define C07_CAL_GAIN {1,1,1,1,1,1,1,1}
#define C07_CAL_OFFSET {0,0,0,0,0,0,0,0}

#ifdef C07_SIMULATION_PROFILE
/* Historical private fixture. Not used to validate the published default. */
#include "simulation_profile.h"
#else
static const c07_commissioning_profile c07_commissioning={.commutation={
    .enabled=true,
    /* Assumed GPIO-to-channel envelope. Selected circuit-model markers fit
     * these values; the real optocoupler LED/receiver response is unmeasured.
     * ON includes the model's 8 V gate marker, not just 0.1 A conduction onset. */
    .hs_off_max_s=400e-9,.hs_on_min_s=0,.hs_on_max_s=300e-9,
    .ls_off_max_s=400e-9,.ls_on_min_s=0,.ls_on_max_s=300e-9,
    .nonoverlap_margin_s=300e-9,
    /* Residual error after calibration, in physical units. */
    .current_error_a=.2,
    .vin_error_v=.03,.vout_error_v=.02,
    .minimum_current_a=-2,.maximum_current_a=24,
    .sync_enter_a=3,.sync_exit_a=2,.sync_entry_dwell_us=2000,
    .input_headroom_v=5
}};
#endif
#endif
