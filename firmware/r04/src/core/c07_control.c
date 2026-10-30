#include "c07_control.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#ifndef C07_SELECTION_DIAGNOSTICS
#define C07_SELECTION_DIAGNOSTICS 1
#endif

#define R(x) ((c07_real_t)(x))
static c07_real_t lo(c07_real_t a,c07_real_t b){return a<b?a:b;}
static c07_real_t hi(c07_real_t a,c07_real_t b){return a>b?a:b;}
static c07_real_t absr(c07_real_t a){return a<R(0)?-a:a;}
static c07_real_t clamp(c07_real_t x,c07_real_t a,c07_real_t b){return hi(a,lo(b,x));}
/* Timers saturate instead of wrapping during indefinite unsettled operation. */
static uint32_t add_time(uint32_t a,uint32_t b){return UINT32_MAX-a<b?UINT32_MAX:a+b;}
static void pi_track(c07_pi *p,c07_real_t output,c07_real_t error,c07_real_t ff){
    p->integral=output-ff-p->kp*error;
}
static c07_real_t pi_step(c07_pi *p,c07_real_t error,c07_real_t dt,
                         c07_real_t lower,c07_real_t upper,c07_real_t ff){
    c07_real_t inc=p->ki*error*dt;
    c07_real_t base=ff+p->kp*error+p->integral;
    /* Consume available headroom instead of discarding a whole increment.
     * When the proportional term alone is already outside the bounds, hold
     * the worsening integral; never move it backwards just to cancel P. */
    if(inc>R(0))inc=lo(inc,hi(R(0),upper-base));
    else if(inc<R(0))inc=hi(inc,lo(R(0),lower-base));
    p->integral+=inc;
    return clamp(ff+p->kp*error+p->integral,lower,upper);
}
static void clear_window(c07_mppt *m){
    m->settled_us=0; m->window_us=0; m->power_sum=R(0); m->voltage_sum=R(0);m->current_sum=R(0);
}
static void mppt_reset(c07_controller *s,c07_real_t voc,c07_real_t minimum){
    c07_mppt *m=&s->mppt; const c07_config *c=&s->cfg;
    memset(m,0,sizeof(*m)); m->maximum=c->MPPT_REFERENCE_MAX_V;
    c07_real_t initial_max=hi(minimum,lo(m->maximum,voc-c->MPPT_VOC_MARGIN_V));
    m->target=clamp(voc*c->MPPT_INITIAL_FRACTION,minimum,initial_max);
    m->direction=1; m->step_v=c->MPPT_STEP_MIN_V;
}
static void mppt_update_legacy(c07_controller *s,c07_real_t voltage,c07_real_t power,
                        c07_real_t minimum,uint32_t dt_us,bool limited,bool settled){
    c07_mppt *m=&s->mppt; const c07_config *c=&s->cfg;
    if(minimum>m->maximum) limited=true;
    c07_real_t bounded=clamp(m->target,minimum,hi(minimum,m->maximum));
    if(bounded!=m->target){
        m->target=bounded; m->have_previous_power=false; m->elapsed_us=0; clear_window(m);
    }
    if(limited){m->have_previous_power=false;m->elapsed_us=0;clear_window(m);return;}
    m->elapsed_us=add_time(m->elapsed_us,dt_us);
    if(!settled){clear_window(m);return;}
    m->settled_us=add_time(m->settled_us,dt_us);
    if(m->settled_us<c->MPPT_SETTLE_US) return;
    m->power_sum+=power*R(dt_us);m->voltage_sum+=voltage*R(dt_us);
    m->window_us=add_time(m->window_us,dt_us);
    if(m->elapsed_us<c->MPPT_PERIOD_US || m->window_us<c->MPPT_WINDOW_US) return;
    c07_real_t mean=m->power_sum/R(m->window_us);
    if(m->have_previous_power){
        c07_real_t change=mean-m->previous_power;
        if(change < -c->POWER_DEADBAND_W) m->direction=-m->direction;
        c07_real_t scale=absr(change)/hi(absr(m->previous_power),c->MPPT_POWER_FLOOR_W);
        m->step_v=clamp(c->MPPT_STEP_MIN_V+c->MPPT_STEP_GAIN_V*scale,
                       c->MPPT_STEP_MIN_V,c->MPPT_STEP_MAX_V);
        /* Keep the power anchor inside the deadband. Replacing it after
         * every tiny step hides a cumulative downhill excursion. */
        if(absr(change)>c->POWER_DEADBAND_W)m->previous_power=mean;
    } else {m->step_v=c->MPPT_STEP_MIN_V;m->previous_power=mean;m->have_previous_power=true;}
    m->target=clamp(m->target+R(m->direction)*m->step_v,minimum,m->maximum);
    if(m->target<=minimum){m->direction=1;m->previous_power=mean;}
    else if(m->target>=m->maximum){m->direction=-1;m->previous_power=mean;}
    m->updates++;m->elapsed_us=0;clear_window(m);
}

/* Selection experiment: both methods receive exactly the same settled raw V/I
 * windows, and compare mean(V)*mean(I). This is intentionally distinct from
 * the preserved legacy mean(VI) path. No model MPP is supplied to either one. */
static c07_real_t selection_step(const c07_config *c,c07_real_t dp,c07_real_t dv,
                                 c07_real_t current,c07_real_t prior_power){
    if(c->MPPT_STEP_MODE==0)return c->MPPT_STEP_MIN_V;
    if(c->MPPT_STEP_MODE==1){
        c07_real_t scale=absr(dp)/hi(absr(prior_power),c->MPPT_POWER_FLOOR_W);
        return clamp(c->MPPT_STEP_MIN_V+c->MPPT_STEP_GAIN_V*scale,
                     c->MPPT_STEP_MIN_V,c->MPPT_STEP_MAX_V);
    }
    if(absr(dv)<=c->MPPT_VOLTAGE_DEADBAND_V)return c->MPPT_STEP_MIN_V;
    c07_real_t normalized=absr(dp/dv)/hi(absr(current),c->MPPT_SLOPE_CURRENT_FLOOR_A);
    return clamp(c->MPPT_STEP_GAIN_V*normalized,c->MPPT_STEP_MIN_V,c->MPPT_STEP_MAX_V);
}
static int inccond_direction(const c07_config *c,c07_real_t voltage,c07_real_t current,
                            c07_real_t dv,c07_real_t di,c07_real_t *signal){
    /* Standard current-point incremental conductance, without division:
     * S=I_k*dV+V_k*dI. sign(dP/dV) approximately sign(S)*sign(dV).
     * S differs from discrete delta(V*I) by dV*dI; that finite-step term is
     * deliberately not hidden by replacing current-point terms by midpoints. */
    *signal=current*dv+voltage*di;
    if(absr(dv)<=c->MPPT_VOLTAGE_DEADBAND_V){
        if(di>c->MPPT_CURRENT_DEADBAND_A)return 1;
        if(di < -c->MPPT_CURRENT_DEADBAND_A)return -1;
        return 0;
    }
    if(absr(*signal)<=c->POWER_DEADBAND_W)return 0;
    return ((*signal>R(0))==(dv>R(0)))?1:-1;
}
static void selection_anchor(c07_mppt *m,c07_real_t v,c07_real_t i,c07_real_t p){
    m->previous_voltage=v;m->previous_current=i;m->previous_power=p;m->have_previous_power=true;
}
static void selection_decide(const c07_config *c,c07_mppt *m,c07_real_t minimum){
    c07_real_t v=m->voltage_sum/R(m->window_us),i=m->current_sum/R(m->window_us),p=v*i;
    m->last_mean_voltage=v;m->last_mean_current=i;m->last_mean_power=p;
    #if C07_SELECTION_DIAGNOSTICS
    m->last_mean_instant_power=m->power_sum/R(m->window_us);
#endif
    m->last_signal=R(0);
    int direction=m->direction;
    if(!m->have_previous_power){m->step_v=c->MPPT_STEP_MIN_V;selection_anchor(m,v,i,p);}
    else {
        c07_real_t dp=p-m->previous_power,dv=v-m->previous_voltage,di=i-m->previous_current;
        m->step_v=selection_step(c,dp,dv,i,m->previous_power);
        if(c->MPPT_METHOD==1){
            m->last_signal=dp;
            if(dp < -c->POWER_DEADBAND_W)direction=-direction;
            /* Continuing perturbations retain the reference power until
             * accumulated change becomes significant. */
            if(absr(dp)>c->POWER_DEADBAND_W)selection_anchor(m,v,i,p);
        } else {
            direction=inccond_direction(c,v,i,dv,di,&m->last_signal);
            /* Retain an anchor while holding, so slow irradiation drift is
             * not forgotten by replacing it on every tiny sample change. */
            if(direction!=0)selection_anchor(m,v,i,p);
        }
    }
    c07_real_t old=m->target;
    if(direction!=0){m->direction=direction;m->target=clamp(old+R(direction)*m->step_v,minimum,m->maximum);}
    if(m->target!=old)m->moves++;else m->holds++;
    if(m->target<=minimum){m->direction=1;selection_anchor(m,v,i,p);}
    else if(m->target>=m->maximum){m->direction=-1;selection_anchor(m,v,i,p);}
    m->updates++;m->elapsed_us=0;clear_window(m);
}
void c07_mppt_invalidate(c07_controller *s,bool clear_anchor){
    s->async.epoch++;
    if(s->async.pending)s->async.invalidated++;
    s->async.pending=false;
    clear_window(&s->mppt);
    if(clear_anchor)s->mppt.have_previous_power=false;
}
bool c07_mppt_begin(c07_controller *s,c07_mppt_job *job){
    if(!s||!job)return false;
    uint32_t seq=atomic_load_explicit(&s->async.request_seq,memory_order_acquire);
    if(!seq||(seq&1u)||seq==s->async.background_seen)return false;
    *job=s->async.request;
    atomic_thread_fence(memory_order_acquire);
    if(seq!=atomic_load_explicit(&s->async.request_seq,memory_order_acquire))return false;
    s->async.background_seen=seq;
    return true;
}
bool c07_mppt_compute(const c07_config *c,const c07_mppt_job *job,c07_mppt_result *result){
    if(!c||!job||!result)return false;
    result->job=*job;result->valid=false;
    if(!job->state.window_us)return false;
    selection_decide(c,&result->job.state,job->minimum);
    c07_mppt *m=&result->job.state;
    result->valid=isfinite(m->target)&&isfinite(m->step_v)&&isfinite(m->previous_power)
        &&isfinite(m->previous_voltage)&&isfinite(m->previous_current)&&isfinite(m->last_signal);
    return result->valid;
}
void c07_mppt_publish(c07_controller *s,const c07_mppt_result *result){
    if(!s||!result)return;
    atomic_store_explicit(&s->async.result_seq,result->job.request_id-1u,memory_order_relaxed);
    atomic_thread_fence(memory_order_release);
    s->async.result=*result;
    atomic_store_explicit(&s->async.result_seq,result->job.request_id,memory_order_release);
}
bool c07_mppt_background(c07_controller *s){
    c07_mppt_job job;c07_mppt_result result;
    if(!c07_mppt_begin(s,&job))return false;
    c07_mppt_compute(&s->cfg,&job,&result);c07_mppt_publish(s,&result);return true;
}
static void mppt_update_selection(c07_controller *s,c07_real_t voltage,c07_real_t current,
                                  c07_real_t minimum,uint32_t dt_us,bool limited,bool settled){
    c07_mppt *m=&s->mppt;const c07_config *c=&s->cfg;c07_mppt_mailbox *a=&s->async;
    if(minimum>m->maximum)limited=true;
    c07_real_t bounded=clamp(m->target,minimum,hi(minimum,m->maximum));
    if(bounded!=m->target){c07_mppt_invalidate(s,true);m->target=bounded;m->elapsed_us=0;}
    if(limited){c07_mppt_invalidate(s,true);m->elapsed_us=0;return;}
    m->elapsed_us=add_time(m->elapsed_us,dt_us);
    if(!settled){c07_mppt_invalidate(s,false);return;}
    /* Result is consumed only at fast-end, after this frame's PI and limits.
     * Main cannot run while this IRQ checks/copies the committed result. */
    uint32_t ready=atomic_load_explicit(&a->result_seq,memory_order_acquire);
    if(ready&&!(ready&1u)&&ready!=a->consumed_result){
        a->consumed_result=ready;
        const volatile c07_mppt_result *r=&a->result;
        bool fresh=a->pending&&ready==a->pending_request&&r->job.epoch==a->epoch
            &&r->job.captured_us<=a->clock_us&&a->clock_us-r->job.captured_us<=c->MPPT_PERIOD_US;
        if(fresh&&r->valid&&r->job.state.target>=minimum&&r->job.state.target<=m->maximum){
            *m=r->job.state;clear_window(m);m->elapsed_us=0;
            a->pending=false;a->applied++;a->epoch++;
            return;
        }
        a->rejected++;
        if(a->pending&&ready==a->pending_request&&r->job.epoch==a->epoch){
            c07_mppt_invalidate(s,true);m->elapsed_us=0;return;
        }
    }
    if(a->pending){
        if(a->clock_us-a->pending_since_us<=c->MPPT_PERIOD_US)return;
        a->expired++;c07_mppt_invalidate(s,true);m->elapsed_us=0;return;
    }
    m->settled_us=add_time(m->settled_us,dt_us);
    if(m->settled_us<c->MPPT_SETTLE_US)return;
    m->voltage_sum+=voltage*R(dt_us);m->current_sum+=current*R(dt_us);
    m->window_us=add_time(m->window_us,dt_us);
    if(m->elapsed_us<c->MPPT_PERIOD_US||m->window_us<c->MPPT_WINDOW_US)return;
    uint32_t id=a->next_request+2u;if(!id)id=2u;a->next_request=id;
    atomic_store_explicit(&a->request_seq,id-1u,memory_order_relaxed);
    atomic_thread_fence(memory_order_release);
    a->request=(c07_mppt_job){.epoch=a->epoch,.captured_us=a->clock_us,.request_id=id,.minimum=minimum,.state=*m};
    atomic_store_explicit(&a->request_seq,id,memory_order_release);
    a->pending=true;a->pending_request=id;a->pending_since_us=a->clock_us;a->published++;
}

void c07_config_defaults(c07_config *c){
    if(!c)return;
    c->MPPT_METHOD=0;c->MPPT_STEP_MODE=1;
#define C07_DEFAULT_REAL(name,value) c->name=R(value);
    C07_CONFIG_REAL_FIELDS(C07_DEFAULT_REAL)
#undef C07_DEFAULT_REAL
#define C07_DEFAULT_TIME(name,value) c->name=(uint32_t)(value);
    C07_CONFIG_TIME_FIELDS(C07_DEFAULT_TIME)
#undef C07_DEFAULT_TIME
}
static bool valid_config(const c07_config *c){
#define C07_FINITE(name,value) if(!isfinite(c->name))return false;
    C07_CONFIG_REAL_FIELDS(C07_FINITE)
#undef C07_FINITE
    return c->MPPT_METHOD<=2 && c->MPPT_STEP_MODE<=2
      && c->MPPT_VOLTAGE_DEADBAND_V>=R(0) && c->MPPT_CURRENT_DEADBAND_A>=R(0)
      && c->MPPT_SLOPE_CURRENT_FLOOR_A>R(0)
      && c->CONTROL_PERIOD_US>0 && c->MAX_LOOP_GAP_US>=c->CONTROL_PERIOD_US
      && c->VOLTAGE_PERIOD_US>=c->CONTROL_PERIOD_US && c->MPPT_PERIOD_US>c->VOLTAGE_PERIOD_US
      && c->DUTY_MIN>R(0) && c->DUTY_MIN<c->DUTY_MAX && c->DUTY_MAX<R(1)
      && c->PV_KP_A_PER_V>=R(0) && c->PV_KI_A_PER_V_S>=R(0)
      && c->PV_KD_A_S_PER_V>=R(0) && c->IL_RISE_A_PER_S>=R(0)
      && c->PV_REFERENCE_WEIGHT>=R(0) && c->PV_REFERENCE_WEIGHT<=R(1)
      && c->IL_KP_DUTY_PER_A>=R(0) && c->IL_KI_DUTY_PER_A_S>=R(0)
      && c->CV_KP_A_PER_V>=R(0) && c->CV_KI_A_PER_V_S>=R(0)
      && c->CHARGE_CURRENT_A>R(0) && c->INPUT_FOLDBACK_BAND_V>R(0)
      && c->TSINK_MAX_C>c->TSINK_DERATE_C && c->MPPT_POWER_FLOOR_W>R(0)
      && c->MPPT_WINDOW_US>0 && c->MPPT_REACQUIRE_US>0
      && c->MPPT_STEP_MIN_V>R(0) && c->MPPT_STEP_MAX_V>=c->MPPT_STEP_MIN_V
      && c->MPPT_INITIAL_FRACTION>R(0) && c->MPPT_INITIAL_FRACTION<R(1);
}
int c07_init(c07_controller *s,const c07_config *c){
    if(!s || !c)return C07_ERR_ARGUMENT;
    memset(s,0,sizeof(*s));
    if(!valid_config(c))return C07_ERR_CONFIG;
    atomic_init(&s->async.request_seq,0);atomic_init(&s->async.result_seq,0);
    s->cfg=*c;
    s->pv_pi.kp=c->PV_KP_A_PER_V;s->pv_pi.ki=c->PV_KI_A_PER_V_S;
    s->current_pi.kp=c->IL_KP_DUTY_PER_A;s->current_pi.ki=c->IL_KI_DUTY_PER_A_S;
    s->cv_pi.kp=c->CV_KP_A_PER_V;s->cv_pi.ki=c->CV_KI_A_PER_V_S;
    c07_reset(s);return C07_OK;
}
void c07_reset(c07_controller *s){
    if(!s)return;
    c07_mppt_invalidate(s,true);
    s->duty=R(0);s->current_target=R(0);s->initialized=false;s->tracking_started=false;
    s->current=R(0);s->output_voltage=R(0);s->pv_voltage=R(0);s->power=R(0);
    s->pv_previous=R(0);s->pv_derivative=R(0);
    s->outer_elapsed_us=0;s->cv_ceiling=s->cfg.CHARGE_CURRENT_A;
    s->limited=true;s->limit_reason=C07_LIMIT_STOPPED;
    s->reacquire_elapsed_us=0;s->reacquisitions=0;s->restart_feedforward_pending=false;s->off_current_hold=false;
    pi_track(&s->pv_pi,R(0),R(0),R(0));pi_track(&s->current_pi,R(0),R(0),R(0));
    pi_track(&s->cv_pi,s->cfg.CHARGE_CURRENT_A,R(0),R(0));mppt_reset(s,R(0),R(0));
}
int c07_start_tracking(c07_controller *s,c07_real_t voc,c07_real_t battery){
    if(!s)return C07_ERR_ARGUMENT;
    if(!isfinite(voc)||!isfinite(battery)){c07_reset(s);return C07_ERR_SAMPLE;}
    const c07_config *c=&s->cfg;
    c07_real_t minimum=hi(battery+c->PV_HEADROOM_V,(battery+c->BUCK_LOSS_MARGIN_V)/c->DUTY_MAX);
    c07_mppt_invalidate(s,true);
    mppt_reset(s,voc,minimum);s->tracking_started=false;s->outer_elapsed_us=0;return C07_OK;
}
static c07_real_t ramp(c07_controller *s,const c07_sample *i,uint32_t us){
    const c07_config *c=&s->cfg;
    c07_real_t target=lo(i->vbat+c->CONNECT_OFFSET_V,lo(c->CHARGE_VOLTAGE_V,c->VOUT_TRIP_V-R(.5)));
    c07_real_t ei=s->current-c->RAMP_CURRENT_A,ev=s->output_voltage-target;
    c07_real_t reduction=hi(hi(R(0),ei)*c->CURRENT_REDUCTION_PER_A_S,
                            hi(R(0),ev)*c->VOLTAGE_REDUCTION_PER_V_S);
    if(reduction>R(0))s->duty-=reduction*R(us)/R(1000000);
    else if(ei < -c->CURRENT_DEADBAND_A && ev < -c->VOLTAGE_DEADBAND_V)
        s->duty+=c->DUTY_RISE_PER_S*R(us)/R(1000000);
    s->duty=clamp(s->duty,R(0),c->DUTY_MAX);s->current_target=c->RAMP_CURRENT_A;
    s->limited=true;s->limit_reason=C07_LIMIT_STARTUP;
    return s->duty>=c->DUTY_MIN?s->duty:R(0);
}
static c07_real_t charge(c07_controller *s,const c07_sample *i,uint32_t us){
    const c07_config *c=&s->cfg;
    c07_real_t dt=R(us)/R(1000000);
    c07_real_t minimum=hi(i->vbat+c->PV_HEADROOM_V,
                         (hi(i->vout,i->vbat)+c->BUCK_LOSS_MARGIN_V)/c->DUTY_MAX);
    c07_real_t ff=clamp(i->vout/hi(i->vin,R(1)),R(0),c->DUTY_MAX);
    c07_real_t ev=s->pv_voltage-s->mppt.target, ecv=c->CHARGE_VOLTAGE_V-hi(i->vbat,s->output_voltage);
    /* Measurement derivative: positive dV/dt asks for more current in a Buck
     * input-voltage loop. No reference derivative or setpoint kick.
     * Extra reference feedforward makes total P = Kp*(V-beta*Vref), while
     * pi_step still integrates the full error V-Vref. Back-calculation uses
     * this same extra term so saturation does not undo reference weighting. */
    c07_real_t pv_d=c->PV_KD_A_S_PER_V*s->pv_derivative
      +c->PV_KP_A_PER_V*(R(1)-c->PV_REFERENCE_WEIGHT)*s->mppt.target;
    if(!s->tracking_started){
        s->current_target=clamp(s->current,R(0),c->CHARGE_CURRENT_A);
        pi_track(&s->pv_pi,s->current_target,ev,pv_d);
        if(s->duty<c->DUTY_MIN){
            /* A cold OFF handover has no learned operating-point bias. */
            pi_track(&s->current_pi,R(0),R(0),R(0));s->off_current_hold=true;
        } else pi_track(&s->current_pi,s->duty,s->current_target-s->current,ff);
        /* Bumpless CV boundary at zero voltage error; positive error keeps
         * proportional current headroom while PV is the active selector. */
        s->cv_pi.integral=s->current_target;s->tracking_started=true;
    }
    s->outer_elapsed_us=add_time(s->outer_elapsed_us,us);s->limited=false;s->limit_reason=C07_LIMIT_MPPT;
    c07_real_t ceiling=c->CHARGE_CURRENT_A;
    bool outer_step=s->outer_elapsed_us>=c->VOLTAGE_PERIOD_US;
    if(outer_step){
        c07_real_t odt=R(s->outer_elapsed_us)/R(1000000);s->outer_elapsed_us=0;
        s->cv_ceiling=pi_step(&s->cv_pi,ecv,odt,R(0),ceiling,R(0));
        c07_real_t wanted=pi_step(&s->pv_pi,ev,odt,R(0),s->cv_ceiling,pv_d);
        s->current_target=c->IL_RISE_A_PER_S>R(0)?lo(wanted,s->current_target+c->IL_RISE_A_PER_S*odt):wanted;
        pi_track(&s->pv_pi,s->current_target,ev,pv_d);
    }
    ceiling=lo(ceiling,s->cv_ceiling);
    if(s->cv_ceiling<c->CHARGE_CURRENT_A-c->CURRENT_DEADBAND_A &&
       s->current_target>=s->cv_ceiling-R(1e-6)){s->limited=true;s->limit_reason=C07_LIMIT_CV;}
    /* 57.1 V is the CV regulation target, not the emergency OFF threshold.
     * The board supervisor independently latches the raw-voltage trip. */
    if(hi(i->vbat,i->vout)>=c->VOUT_TRIP_V){ceiling=R(0);s->limited=true;s->limit_reason=C07_LIMIT_VOLTAGE;}
    if(i->ipv>=c->INPUT_CURRENT_LIMIT_A){
        ceiling=lo(ceiling,hi(R(0),s->current_target-c->INPUT_LIMIT_REDUCTION_A_PER_S*dt));
        s->limited=true;s->limit_reason=C07_LIMIT_INPUT_CURRENT;
    }
    c07_real_t margin=i->vin-i->vbat;
    if(margin<c->PV_HEADROOM_V+c->INPUT_FOLDBACK_BAND_V){
        c07_real_t fraction=clamp((margin-c->PV_HEADROOM_V)/c->INPUT_FOLDBACK_BAND_V,R(0),R(1));
        c07_real_t cap=c->CHARGE_CURRENT_A*fraction;ceiling=lo(ceiling,cap);
        if(s->current_target>=cap-c->CURRENT_DEADBAND_A){s->limited=true;s->limit_reason=C07_LIMIT_INPUT_FOLDBACK;}
    }
    if(minimum>s->mppt.maximum){ceiling=R(0);s->limited=true;s->limit_reason=C07_LIMIT_BUCK_UNREACHABLE;}
    if(i->tsink>c->TSINK_DERATE_C){
        c07_real_t fraction=clamp((c->TSINK_MAX_C-i->tsink)/(c->TSINK_MAX_C-c->TSINK_DERATE_C),R(0),R(1));
        c07_real_t cap=c->CHARGE_CURRENT_A*fraction;ceiling=lo(ceiling,cap);
        if(s->current_target>=cap-c->CURRENT_DEADBAND_A){s->limited=true;s->limit_reason=C07_LIMIT_THERMAL;}
    }
    if(s->current_target>=c->CHARGE_CURRENT_A-c->CURRENT_DEADBAND_A){s->limited=true;s->limit_reason=C07_LIMIT_CURRENT;}
    bool reduced=s->current_target>ceiling;
    if(reduced){s->current_target=hi(R(0),ceiling);pi_track(&s->pv_pi,s->current_target,ev,pv_d);}
    /* Inactive CV tracks the selected current at zero CV error. This retains
     * proportional voltage headroom instead of cancelling it each tick and
     * accidentally imposing Ki_cv*voltage_error as a normal-current slew.
     * An active CV PI keeps its own integral; a separate safety selector can
     * still back-calculate it when that selector reduces the current further. */
    if(outer_step||reduced){
        if(s->current_target<s->cv_ceiling-R(1e-6))s->cv_pi.integral=s->current_target;
        else if(reduced)pi_track(&s->cv_pi,s->current_target,ecv,R(0));
    }
    c07_real_t ei=s->current_target-s->current;
    if(s->current_target<=c->ZERO_CURRENT_TARGET_A){
        /* OFF is outside the continuous control range: hold the learned
         * integral rather than forcing a -feedforward bias. In light-load
         * DCM, that learned bias is needed on the next positive request. */
        s->duty=R(0);s->off_current_hold=true;
    }
    else if((s->off_current_hold||s->restart_feedforward_pending) &&
            s->current>hi(c->MPPT_NOLOAD_A,s->current_target+c->CURRENT_DEADBAND_A)){
        /* Let residual L1 energy decay before resuming a smaller current. */
        s->duty=R(0);
        s->limited=true;s->limit_reason=C07_LIMIT_CURRENT;
    }
    else {
        s->off_current_hold=false;
        if(s->restart_feedforward_pending){pi_track(&s->current_pi,R(0),R(0),R(0));s->restart_feedforward_pending=false;}
        s->duty=pi_step(&s->current_pi,ei,dt,R(0),c->DUTY_MAX,ff);
        if(s->duty>R(0) && s->duty<c->DUTY_MIN){s->duty=R(0);s->limited=true;s->limit_reason=C07_LIMIT_MIN_DUTY;}
        else if(s->duty>=c->DUTY_MAX){
            s->limited=true;s->limit_reason=C07_LIMIT_MAX_DUTY;
            pi_track(&s->pv_pi,lo(s->current_target,hi(R(0),s->current)),ev,pv_d);
        }
    }
    bool pair_ok=i->pv_pair_span_us>=0 && (uint32_t)i->pv_pair_span_us<=c->MPPT_MAX_PAIR_SPAN_US;
    bool reacquire=pair_ok && s->duty==R(0) && s->current_target<=c->ZERO_CURRENT_TARGET_A
      && absr(i->il)<c->MPPT_NOLOAD_A && absr(i->ipv)<c->MPPT_NOLOAD_A
      && s->pv_voltage<s->mppt.target-c->MPPT_SETTLE_V && i->vin>minimum
      && (s->limit_reason==C07_LIMIT_MPPT || s->limit_reason==C07_LIMIT_MIN_DUTY);
    s->reacquire_elapsed_us=reacquire?add_time(s->reacquire_elapsed_us,us):0;
    if(s->reacquire_elapsed_us>=c->MPPT_REACQUIRE_US){
        c07_mppt_invalidate(s,true);
        mppt_reset(s,i->vpv,minimum);ev=s->pv_voltage-s->mppt.target;
        pv_d=c->PV_KD_A_S_PER_V*s->pv_derivative
          +c->PV_KP_A_PER_V*(R(1)-c->PV_REFERENCE_WEIGHT)*s->mppt.target;
        pi_track(&s->pv_pi,R(0),ev,pv_d);pi_track(&s->current_pi,R(0),R(0),R(0));
        s->restart_feedforward_pending=true;s->reacquire_elapsed_us=0;s->reacquisitions++;
    }
    bool settled=absr(ev)<=c->MPPT_SETTLE_V && absr(ei)<=c->MPPT_SETTLE_A;
    if(c->MPPT_METHOD==0)mppt_update_legacy(s,s->pv_voltage,s->power,minimum,us,s->limited||!pair_ok,settled);
    else mppt_update_selection(s,i->vpv,i->ipv,minimum,us,s->limited||!pair_ok,settled);
    return s->duty;
}
int c07_step(c07_controller *s,const c07_sample *i,uint32_t us,bool charging,c07_real_t *out){
    if(out)*out=R(0);
    if(!s||!i||!out)return C07_ERR_ARGUMENT;
    s->async.clock_us+=us;
    if(!charging)c07_mppt_invalidate(s,true);
    if(!us||us>s->cfg.MAX_LOOP_GAP_US){c07_reset(s);return C07_ERR_DT;}
    if(!i->valid||!isfinite(i->il)||!isfinite(i->ipv)||!isfinite(i->vpv)||!isfinite(i->vin)
       ||!isfinite(i->vout)||!isfinite(i->vbat)||!isfinite(i->tsink)){
        c07_reset(s);return C07_ERR_SAMPLE;
    }
    c07_real_t ai=s->initialized?R(us)/(R(s->cfg.IL_FILTER_US)+R(us)):R(1);
    c07_real_t av=s->initialized?R(us)/(R(s->cfg.VOLTAGE_FILTER_US)+R(us)):R(1);
    s->current+=ai*(i->il-s->current);s->output_voltage+=av*(i->vout-s->output_voltage);
    s->pv_voltage+=av*(i->vpv-s->pv_voltage);s->power=i->vpv*i->ipv;
    if(s->initialized){
        c07_real_t raw=(s->pv_voltage-s->pv_previous)*R(1000000)/R(us);
        c07_real_t ad=R(us)/(R(s->cfg.PV_D_FILTER_US)+R(us));
        s->pv_derivative+=ad*(raw-s->pv_derivative);
    }else s->pv_derivative=R(0);
    s->pv_previous=s->pv_voltage;s->initialized=true;
    if(!isfinite(s->current)||!isfinite(s->output_voltage)||!isfinite(s->pv_voltage)||!isfinite(s->power)
       ||!isfinite(s->pv_derivative)||!isfinite(s->pv_previous)){
        c07_reset(s);return C07_ERR_NUMERIC;
    }
    c07_real_t duty=charging?charge(s,i,us):ramp(s,i,us);
    if(!isfinite(duty)||!isfinite(s->current_target)||!isfinite(s->pv_pi.integral)
       ||!isfinite(s->current_pi.integral)||!isfinite(s->cv_pi.integral)
       ||!isfinite(s->mppt.target)||!isfinite(s->mppt.power_sum)||!isfinite(s->mppt.voltage_sum)){
        c07_reset(s);return C07_ERR_NUMERIC;
    }
    *out=duty;return C07_OK;
}
const char *c07_limit_name(c07_limit r){
    static const char *const names[]={"STOPPED","STARTUP","MPPT","CV","VOLTAGE_LIMIT",
      "INPUT_CURRENT","INPUT_FOLDBACK","BUCK_UNREACHABLE","THERMAL","CURRENT_LIMIT","MIN_DUTY","MAX_DUTY"};
    return (unsigned)r<sizeof(names)/sizeof(names[0])?names[r]:"UNKNOWN";
}
