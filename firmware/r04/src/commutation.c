#include "commutation.h"
#include "board_config.h"
#include <math.h>
#include <string.h>

const char *c07_commutation_validate(const c07_commutation_config *c){
    if(!c)return "COMMUTATION_CONFIG";
    if(!c->enabled)return "COMMUTATION_DISABLED";
#define NONNEG(x) if(!isfinite(c->x)||c->x<0)return "COMMUTATION_CONFIG"
    NONNEG(hs_off_max_s);NONNEG(hs_on_min_s);NONNEG(hs_on_max_s);
    NONNEG(ls_off_max_s);NONNEG(ls_on_min_s);NONNEG(ls_on_max_s);
    NONNEG(nonoverlap_margin_s);NONNEG(current_error_a);
    NONNEG(vin_error_v);NONNEG(vout_error_v);
    NONNEG(sync_enter_a);NONNEG(sync_exit_a);NONNEG(input_headroom_v);
#undef NONNEG
    if(!isfinite(c->minimum_current_a)||!isfinite(c->maximum_current_a)
       ||c->minimum_current_a>=0||c->maximum_current_a<=c->sync_enter_a
       ||c->sync_enter_a<=c->sync_exit_a||c->sync_exit_a<=0
       ||!c->sync_entry_dwell_us
       ||c->nonoverlap_margin_s<=0||c->hs_on_min_s>c->hs_on_max_s
       ||c->ls_on_min_s>c->ls_on_max_s)
        return "COMMUTATION_CONFIG";
    const double hl=(double)C07_PIO_DEAD_HL_TICKS/C07_CPU_HZ;
    const double lh=(double)C07_PIO_DEAD_LH_TICKS/C07_CPU_HZ;
    if(hl+c->ls_on_min_s-c->hs_off_max_s<c->nonoverlap_margin_s
       ||lh+c->hs_on_min_s-c->ls_off_max_s<c->nonoverlap_margin_s)
        return "GATE_TIMING_CONFIG";
    return NULL;
}
void c07_commutation_reset(c07_commutation *s){memset(s,0,sizeof(*s));}
const char *c07_commutation_plan(c07_commutation *s,const c07_commutation_config *c,
                                const c07_commutation_input *i,c07_gate_plan *o){
    if(!s||!i||!o)return "COMMUTATION_ARGUMENT";
    memset(o,0,sizeof(*o));
    if(i->duty==0){c07_commutation_reset(s);return NULL;}
    const char *error=c07_commutation_validate(c);if(error)return error;
    if(s->clock_valid&&i->now_us<=s->last_us)return "COMMUTATION_CLOCK";
    s->clock_valid=true;s->last_us=i->now_us;
    if(!isfinite(i->duty)||i->duty<C07_DUTY_MIN||i->duty>C07_DUTY_MAX
       ||!isfinite(i->vin)||!isfinite(i->vout)||!isfinite(i->il))return "COMMUTATION_INPUT";
    if(i->vin-c->vin_error_v<=i->vout+c->vout_error_v+c->input_headroom_v)
        return "INPUT_HEADROOM_LOST";
    double lower=i->il-c->current_error_a,upper=i->il+c->current_error_a;
    if(lower<c->minimum_current_a||upper>c->maximum_current_a)return "CURRENT_OPERATING_RANGE";
    unsigned high=(unsigned)(i->duty*C07_PWM_TICKS);
    if(!high||(double)high/C07_CPU_HZ<=c->hs_on_max_s)return "HS_PULSE_TOO_SHORT";
    if(!i->charging||lower<=c->sync_exit_a){s->synchronous=false;s->entry_pending=false;}
    else if(!s->synchronous){
        if(lower<c->sync_enter_a)s->entry_pending=false;
        else if(!s->entry_pending){s->entry_pending=true;s->entry_since_us=i->now_us;}
        else if(i->now_us<s->entry_since_us)return "COMMUTATION_CLOCK";
        else if(i->now_us-s->entry_since_us>=c->sync_entry_dwell_us)s->synchronous=true;
    }
    unsigned low=0;
    if(s->synchronous&&high+259u<C07_PWM_TICKS){
        low=C07_PWM_TICKS-high-259u;
        if(low<2||(double)low/C07_CPU_HZ<=c->ls_on_max_s)low=0;
    }
    if(!low)s->synchronous=false;
    /* Commutation validates electrical timing, not junction temperature.
     * A one-sample diode-power proxy has no justified hard-trip threshold:
     * heat depends on power history, losses in each device and thermal paths.
     * U5 temperature derating is handled by the regulator/supervisor; offline
     * loss/thermal analysis must include diode mode and packet transitions. */
    o->high_ticks=high;o->low_ticks=low;o->synchronous=low!=0;
    return NULL;
}
