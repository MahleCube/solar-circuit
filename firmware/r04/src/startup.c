#include "startup.h"
#include "board_config.h"
#include "commissioning.h"
#include <math.h>
#include <string.h>

static void clear_command(c07_command *c) {
    c->bias_enable=c->pv_enable=c->battery_enable=c->pv_rearm=c->synchronous_allowed=false;
    c->duty=0;
}
static void enter(c07_charger *c, c07_state state, uint64_t now) {
    c->state=state; c->entered_us=now; c->stable_valid=false;
    if (state==C07_PV_CONNECT) { c->rearm_started=false; c->rearm_finished=false; }
}
static bool stable(c07_charger *c, bool condition, uint64_t now, uint32_t duration) {
    if (!condition) { c->stable_valid=false; return false; }
    if (!c->stable_valid) { c->stable_valid=true; c->stable_since_us=now; }
    return now-c->stable_since_us>=duration;
}
static bool connected_voltage_stable(const c07_sample *s){
    /* BAT_EN permits the LTC4364, not forced closure of a bilateral switch.
     * Its ideal diode blocks a charged battery from an empty Cout. This is
     * an operating-path check, not a precision preconnection voltage window. */
    return fabs(s->vout-s->vbat)<=C07_RUN_DELTA_V;
}
static bool input_available(const c07_sample *s){
    /* Also check residual Cin: battery headroom alone cannot exclude a
     * panel below the disconnected input capacitor. */
    return s->vpv>s->vbat+C07_HEADROOM_V+C07_INPUT_RESTART_MARGIN_V
        &&s->vpv>s->vin+C07_PV_CONNECT_MARGIN_V;
}
int c07_charger_init(c07_charger *c, const c07_config *config) {
    if (!c || !config) return -1;
    memset(c,0,sizeof(*c)); c->state=C07_IDLE; c->fault="";
    return c07_init(&c->regulator,config);
}
void c07_charger_trip(c07_charger *c, const char *reason, uint64_t now) {
    if (c->state!=C07_FAULT) { c->fault=reason; enter(c,C07_FAULT,now); }
    clear_command(&c->command); c07_reset(&c->regulator);
}
void c07_charger_wait_input(c07_charger *c,uint64_t now){
    if(c->state==C07_FAULT)return;
    enter(c,C07_WAIT_INPUT,now);clear_command(&c->command);c07_reset(&c->regulator);
}
const char *c07_measurement_fault(const c07_inputs *i) {
    const c07_sample *s=&i->sample;
    if (!s->valid) return i->error ? i->error : "MEASUREMENT_INVALID";
    if (i->age_us>C07_MAX_SAMPLE_AGE_US) return "STALE_SAMPLE";
    if (!isfinite(s->vin)||!isfinite(s->vpv)||!isfinite(s->ipv)||!isfinite(s->il)
        ||!isfinite(s->vout)||!isfinite(s->vbat)||!isfinite(s->tbat)||!isfinite(s->tsink))
        return "MEASUREMENT_NONFINITE";
    if (!(s->vin>=0 && s->vin<C07_VIN_TRIP_V)) return "VIN_RANGE";
    if (!(s->vpv>=0 && s->vpv<C07_VPV_TRIP_V)) return "VPV_RANGE";
    /* Upper operating targets belong to regulation. A sample above 15 A
     * alone is not a latched fault. board.c rejects ADC rails; hardware fault
     * inputs below remain independent. Configured signed-current limits are
     * operating-envelope checks, not cycle-by-cycle current protection. */
    if (!(s->ipv>C07_REVERSE_TRIP_A)) return "IPV_RANGE";
    double il_min=c07_commissioning.commutation.enabled
        ?c07_commissioning.commutation.minimum_current_a:C07_REVERSE_TRIP_A;
    if (!(s->il>il_min)) return "IL_RANGE";
    if(c07_commissioning.commutation.enabled&&s->il>=c07_commissioning.commutation.maximum_current_a)
        return "IL_RANGE";
    if (!(s->vout>=0 && s->vout<C07_VOUT_TRIP_V)) return "VOUT_RANGE";
    /* Nominal conversion reports the ADC bin's lower edge. A physical 42V
     * sample is code 1481 = 41.9917V; retain that bin, not only code 1482.
     * This one-LSB allowance is quantization, not a general calibration waiver. */
    if (!(s->vbat+C07_VOLTAGE_LSB_V>=C07_VBAT_MIN_V && s->vbat<C07_VBAT_TRIP_V)) return "VBAT_RANGE";
    if (!(s->tbat>=C07_TBAT_MIN_C && s->tbat<C07_TBAT_MAX_C)) return "TBAT_RANGE";
    if (!(s->tsink>=-40 && s->tsink<C07_TSINK_MAX_C)) return "TSINK_RANGE";
    return NULL;
}
const c07_command *c07_charger_step(c07_charger *c, const c07_inputs *i,
                                    const c07_request *r, uint64_t now) {
    const c07_sample *s=&i->sample;
    c07_command *o=&c->command;
    clear_command(o); o->sample_us=i->sample_us;
    uint64_t dt=c->previous_valid ? now-c->previous_us : C07_LOOP_US;
    bool clock_bad=c->previous_valid && now<=c->previous_us;
    c->previous_valid=true; c->previous_us=now;
    bool reset_edge=r->reset_fault&&!c->reset_was_requested;
    c->reset_was_requested=r->reset_fault;
    const char *reason=c07_measurement_fault(i);
    if (clock_bad || dt==0 || dt>C07_MAX_LOOP_GAP_US) reason="LOOP_DEADLINE";
    if (!reason&&i->battery_fault) reason="BATTERY_FAULT";
    if (!reason&&i->pv_fault) reason="PV_OVERCURRENT";
    if(!reason&&c->state>=C07_RAMP_OUTPUT&&c->state<=C07_CHARGING&&!i->bias_good)
        reason="BIAS_LOST";
    if(!reason&&i->pwm_events&&i->pwm_events!=3u)
        reason=(i->pwm_events&4u)?"PIO_LEASE_EXPIRED":"PIO_INTERLOCK";
    /* Input collapse is different from a normal current-decay stop. Disable
     * all paths and let the protection ICs enforce their reverse blocking. */
    if (c->state==C07_FAULT) {
        if (reset_edge&&!r->run&&!reason&&i->pv_stopped) { c->fault=""; enter(c,C07_IDLE,now); }
        return o;
    }
    if (reason) { c07_charger_trip(c,reason,now); return o; }
    if((c->state>=C07_BIAS&&c->state<=C07_STOPPING)
       &&(i->pv_stopped||i->pwm_events==3u)){
        c07_charger_wait_input(c,now);return o;
    }
    if(c->state>=C07_RAMP_OUTPUT&&c->state<=C07_STOPPING
       &&s->vin<=s->vout+C07_HEADROOM_V){
        c07_charger_wait_input(c,now);return o;
    }
    if (!r->run && c->state!=C07_IDLE && c->state!=C07_STOPPING) {
        if (c->state==C07_RAMP_OUTPUT||c->state==C07_CONNECT_BATTERY||c->state==C07_CHARGING) {
            c->stop_battery_enabled=true;
            enter(c,C07_STOPPING,now);
        } else enter(c,C07_IDLE,now);
    }
    uint64_t elapsed=now-c->entered_us;
    switch(c->state) {
    case C07_IDLE:
        c07_reset(&c->regulator);
        if (r->run && input_available(s)
            && s->vbat<C07_CHARGE_VOLTAGE_V) {
            c->open_voltage=s->vpv; enter(c,C07_CLEAR,now);
        } break;
    case C07_CLEAR:
        if (elapsed>=C07_CLEAR_US) {
            if (!i->pv_stopped) c07_charger_trip(c,"PV_CLEAR_FAILED",now);
            else if(!input_available(s))c07_charger_wait_input(c,now);
            else enter(c,C07_PV_CONNECT,now);
        } break;
    case C07_PV_CONNECT:
        if(!c->rearm_started&&!input_available(s)){
            c07_charger_wait_input(c,now);break;
        }
        if (!c->rearm_started&&elapsed>=C07_LOOP_US) { c->rearm_started=true; c->rearm_started_us=now; }
        if (c->rearm_started&&now-c->rearm_started_us>=C07_REARM_US) c->rearm_finished=true;
        if (stable(c,!i->pv_stopped&&s->vin>s->vbat+C07_HEADROOM_V&&c->rearm_finished,
                   now,C07_INPUT_STABLE_US)) enter(c,C07_BIAS,now);
        else if (elapsed>=C07_PV_TIMEOUT_US) c07_charger_wait_input(c,now);
        break;
    case C07_BIAS:
        if (stable(c,i->bias_good,now,C07_BIAS_STABLE_US)) {
            c07_reset(&c->regulator); enter(c,C07_RAMP_OUTPUT,now);
        } else if (elapsed>=C07_BIAS_TIMEOUT_US) c07_charger_trip(c,"BIAS_TIMEOUT",now);
        break;
    case C07_RAMP_OUTPUT:
        if (stable(c,connected_voltage_stable(s),now,C07_CONNECT_STABLE_US))
            enter(c,C07_CONNECT_BATTERY,now);
        else if (elapsed>=C07_RAMP_TIMEOUT_US) c07_charger_trip(c,"OUTPUT_RAMP_TIMEOUT",now);
        break;
    case C07_CONNECT_BATTERY:
        /* The UV-release ramp can temporarily exceed Vbat+1V before HGATE
         * rises. Keep its low-current/voltage cap, reset the settling window,
         * and wait for the normal IC path; do not misclassify that as a fault.
         * This settling policy is not an independent failed-IC detector. */
        if (stable(c,connected_voltage_stable(s),now,C07_BATTERY_READY_US)) {
            if (c07_start_tracking(&c->regulator,c->open_voltage,s->vbat)!=0)
                c07_charger_trip(c,"CONTROL_START",now);
            else enter(c,C07_CHARGING,now);
        } else if (elapsed>=C07_CONNECT_TIMEOUT_US) c07_charger_trip(c,"BATTERY_CONNECT_TIMEOUT",now);
        break;
    case C07_CHARGING:
        if (fabs(s->vout-s->vbat)>C07_RUN_DELTA_V) c07_charger_trip(c,"BATTERY_PATH_VOLTAGE",now);
        break;
    case C07_STOPPING:
        if (stable(c,fabs(s->il)<=C07_STOP_CURRENT_A,now,C07_STOP_STABLE_US)) {
            enter(c,C07_IDLE,now); c07_reset(&c->regulator);
        } else if (elapsed>=C07_STOP_TIMEOUT_US) c07_charger_trip(c,"STOP_CURRENT_TIMEOUT",now);
        break;
    case C07_WAIT_INPUT:
        /* Ordinary source shortage disconnects the battery immediately.
         * Retry only after sustained open-circuit headroom, through CLEAR,
         * PV rearm, BIAS and the output ramp. Hardware faults stay latched. */
        if(stable(c,r->run&&i->pv_stopped&&input_available(s),
           now,C07_INPUT_RETRY_US)){
            c->open_voltage=s->vpv;enter(c,C07_CLEAR,now);
        }
        break;
    case C07_FAULT: break;
    }
    bool power_state=c->state>=C07_BIAS&&c->state<=C07_CHARGING;
    bool switching_state=c->state>=C07_RAMP_OUTPUT&&c->state<=C07_CHARGING;
    if (power_state&&i->pv_stopped) { c07_charger_wait_input(c,now);return o; }
    if (switching_state&&!i->bias_good) c07_charger_trip(c,"BIAS_LOST",now);
    if (c->state==C07_FAULT) return o;
    o->pv_enable=c->state>=C07_PV_CONNECT&&c->state<=C07_STOPPING;
    o->pv_rearm=c->state==C07_PV_CONNECT&&c->rearm_started&&!c->rearm_finished;
    o->bias_enable=c->state>=C07_BIAS&&c->state<=C07_STOPPING;
    o->battery_enable=switching_state;
    o->synchronous_allowed=c->state==C07_CHARGING;
    if (c->state==C07_STOPPING) o->battery_enable=c->stop_battery_enabled;
    else if (switching_state) {
        if (c07_step(&c->regulator,s,(uint32_t)dt,c->state==C07_CHARGING,&o->duty)!=0)
            c07_charger_trip(c,"CONTROL_INVALID",now);
    }
    return o;
}
const char *c07_state_name(c07_state state) {
    static const char *names[]={"IDLE","CLEAR","PV_CONNECT","BIAS","RAMP_OUTPUT",
        "CONNECT_BATTERY","CHARGING","STOPPING","FAULT","WAIT_INPUT"};
    return (unsigned)state<=(unsigned)C07_WAIT_INPUT ? names[(unsigned)state] : "INVALID";
}
