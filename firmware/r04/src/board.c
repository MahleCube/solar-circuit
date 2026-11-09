#include "board.h"
#include "board_config.h"
#include "adc_convert.h"
#include "commissioning.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/time.h"
#include <math.h>
#include <string.h>

/* Per-channel physical-unit calibration: measured_nominal*gain+offset. */
static const c07_real_t cal_gain[8]=C07_CAL_GAIN;
static const c07_real_t cal_offset[8]=C07_CAL_OFFSET;
/* Finish slow monitoring first; keep the PV pair adjacent and IL last. */
static const uint8_t adc_order[8]={4,5,6,2,0,7,1,3};
static const unsigned output_pins[]={C07_PWM_H,C07_PWM_L,C07_REARM,C07_PV_RUN,C07_BAT_EN,C07_BIAS_EN};

static void output_low(unsigned pin) {
    /* Set the SIO output latch before switching the pad away from PIO. */
    gpio_put(pin,0); gpio_set_function(pin,GPIO_FUNC_SIO); gpio_set_dir(pin,GPIO_OUT);
}
static void status(c07_board *b, bool merge) {
    /* One GPIO snapshot; faults observed earlier in this ADC/apply frame do
     * not disappear merely because a pin recovers before the supervisor acts. */
    uint32_t pins=gpio_get_all();
    bool bias_good=(pins&(1u<<C07_BIAS_PG))==0;
    bool battery_fault=(pins&(1u<<C07_BAT_FLT_N))==0;
    bool pv_fault=(pins&(1u<<C07_PV_FLT_N))==0;
    bool pv_stopped=(pins&(1u<<C07_PV_STOPPED))!=0;
    /* Once this frame intentionally disabled BIAS, a later stop call must not
     * reinterpret the resulting PG loss as the cause of the earlier stop. */
    if(!merge)b->inputs.bias_good=bias_good;
    else if(b->bias_enabled)b->inputs.bias_good=b->inputs.bias_good&&bias_good;
    b->inputs.battery_fault=merge?(b->inputs.battery_fault||battery_fault):battery_fault;
    b->inputs.pv_fault=merge?(b->inputs.pv_fault||pv_fault):pv_fault;
    b->inputs.pv_stopped=merge?(b->inputs.pv_stopped||pv_stopped):pv_stopped;
    b->inputs.pwm_events|=c07_pwm_engine_events(&b->pwm);
}
static bool pending_pio_fault(c07_board *b) {
    b->inputs.pwm_events|=c07_pwm_engine_events(&b->pwm);
    return b->inputs.pwm_events!=0;
}
static uint32_t pwm_off(c07_board *b) {
    uint32_t events=c07_pwm_engine_stop(&b->pwm);
    b->inputs.pwm_events|=events;
    b->pwm_enabled=false;c07_commutation_reset(&b->commutation);
    return b->inputs.pwm_events;
}
void c07_board_stop(c07_board *b) {
    if(b->initialized)status(b,true);
    pwm_off(b);
    output_low(C07_REARM); output_low(C07_PV_RUN);
    output_low(C07_BAT_EN); output_low(C07_BIAS_EN);
    b->bias_enabled=b->pv_enabled=b->battery_enabled=false;
    b->rearm.high=false;
}
int c07_board_init(c07_board *b) {
    memset(b,0,sizeof(*b));
    b->configuration_error=c07_commutation_validate(&c07_commissioning.commutation);
    for(unsigned n=0;n<sizeof(output_pins)/sizeof(output_pins[0]);n++) output_low(output_pins[n]);
    if (clock_get_hz(clk_sys)!=C07_CPU_HZ) return -1;
    unsigned inputs[]={C07_BIAS_PG,C07_BAT_FLT_N,C07_PV_FLT_N,C07_PV_STOPPED};
    for(unsigned n=0;n<4;n++) { gpio_init(inputs[n]); gpio_set_dir(inputs[n],GPIO_IN); }
    gpio_pull_up(C07_BIAS_PG); gpio_pull_up(C07_BAT_FLT_N); gpio_pull_up(C07_PV_FLT_N);
    /* PV_STOPPED polarity/pull is owned by the circuit, matching the saved contract. */
    gpio_disable_pulls(C07_PV_STOPPED);
    if(c07_pwm_engine_init(&b->pwm)!=0)return -2;
    b->initialized=true;
    spi_init(spi0,C07_SPI_HZ);
    spi_set_format(spi0,8,SPI_CPOL_0,SPI_CPHA_0,SPI_MSB_FIRST);
    gpio_set_function(C07_SPI_MISO,GPIO_FUNC_SPI);
    gpio_set_function(C07_SPI_MOSI,GPIO_FUNC_SPI);
    gpio_set_function(C07_SPI_SCK,GPIO_FUNC_SPI);
    gpio_init(C07_SPI_CS); gpio_put(C07_SPI_CS,1); gpio_set_dir(C07_SPI_CS,GPIO_OUT);
    b->inputs.error="NOT_SAMPLED"; b->inputs.sample.pv_pair_span_us=-1;
    return 0;
}
static const char *adc_physical(const c07_board *b,unsigned ch,c07_real_t *value) {
    c07_real_t nominal=c07_adc_nominal(ch,b->raw[ch]);
    if (!isfinite(cal_gain[ch])||cal_gain[ch]<=0||!isfinite(cal_offset[ch]))
        return "CAL_INVALID";
    /* Identity calibration needs no software floating-point multiply/add. */
    *value=(cal_gain[ch]==1&&cal_offset[ch]==0)
        ?nominal:nominal*cal_gain[ch]+cal_offset[ch];
    return isfinite(*value)?NULL:"ADC_NONFINITE";
}
const c07_inputs *c07_board_read(c07_board *b) {
    c07_inputs *i=&b->inputs;
    i->sample.valid=false; i->error=NULL; i->completed_us=0;
    /* The previous frame has been consumed by the supervisor/apply path.
     * FAULT state retention belongs to the supervisor, not a resettable pin. */
    i->pwm_events=0;status(b,false);
    i->sample.pv_pair_span_us=-1;
    i->sample_us=time_us_64();
    /* A successful one-word submission is consumed at the next 20 us
     * boundary. Only identify it with this ADC frame after that whole bound
     * plus one microsecond for integer-clock quantization;
     * a requested duty or a command still awaiting the latch is not feedback.
     * This confirms the PIO command, not a measurement of physical gate drive. */
    i->sample.applied_pwm_valid=b->pwm_enabled&&b->pwm.enabled
        &&b->pwm_accepted&&i->sample_us>=b->last_plan_submitted_us+1000000u/C07_PWM_HZ+1u;
    i->sample.applied_pwm_known_off=!b->pwm_enabled&&!b->pwm.enabled;
    i->sample.applied_duty=i->sample.applied_pwm_valid
        ?(c07_real_t)b->last_plan.high_ticks/(c07_real_t)C07_PWM_TICKS:0;
    i->sample.applied_synchronous=i->sample.applied_pwm_valid&&b->last_plan.synchronous;
    c07_real_t physical[8];
    for(unsigned n=0;n<8;n++) { i->adc_start_us[n]=UINT32_MAX; i->adc_end_us[n]=UINT32_MAX; }
    for(unsigned n=0;n<8;n++) {
        unsigned channel=adc_order[n];
        uint8_t tx[3]={(uint8_t)(0x06u|(channel>>2)),(uint8_t)((channel&3u)<<6),0};
        uint8_t rx[3];
        i->adc_start_us[channel]=(uint32_t)(time_us_64()-i->sample_us);
        gpio_put(C07_SPI_CS,0);
        int count=spi_write_read_blocking(spi0,tx,rx,3);
        gpio_put(C07_SPI_CS,1);
        i->adc_end_us[channel]=(uint32_t)(time_us_64()-i->sample_us);
        if (count!=3) { i->error="SPI_LENGTH"; break; }
        b->raw[channel]=(uint16_t)(((rx[1]&15u)<<8)|rx[2]);
        if (b->raw[channel]>=4090) { i->error="ADC_SATURATION"; break; }
        if ((channel==1||channel==3||channel==4||channel==5)&&b->raw[channel]<=8) {
            i->error="ADC_SENSOR_LOW"; break;
        }
        /* Convert earlier channels before the control measurements. Defer
         * Vpv conversion until Ipv has been read, preserving their pair span. */
        if(channel==7)continue;
        if(channel==1){
            i->error=adc_physical(b,7,&physical[7]);
            if(i->error)break;
        }
        i->error=adc_physical(b,channel,&physical[channel]);
        if(i->error)break;
    }
    if (!i->error) {
        i->sample.vin=physical[0]; i->sample.ipv=physical[1];
        i->sample.vout=physical[2]; i->sample.il=physical[3];
        i->sample.tbat=physical[4]; i->sample.tsink=physical[5];
        i->sample.vbat=physical[6]; i->sample.vpv=physical[7];
    }
    status(b,true);
    if(i->pwm_events||!i->bias_good||i->pv_fault||i->battery_fault||i->pv_stopped){
        i->sample.applied_pwm_valid=false;
        i->sample.applied_pwm_known_off=false;
    }
    uint64_t completed=time_us_64();
    i->age_us=(uint32_t)(completed-i->sample_us);
    if (!i->error&&i->age_us>C07_MAX_SAMPLE_AGE_US) i->error="ADC_DEADLINE";
    if (i->error) { c07_board_stop(b); return i; }
    i->sample.pv_pair_span_us=(int32_t)(i->adc_end_us[1]-i->adc_start_us[7]);
    i->frame_id=++b->completed_frames; i->completed_us=completed; i->sample.valid=true;
    return i;
}
static const char *pwm_apply(c07_board *b, c07_real_t duty, uint64_t sample_us,bool charging) {
    if(duty==0)return pwm_off(b)?"PIO_INTERLOCK":NULL;
    if(pending_pio_fault(b))return "PIO_INTERLOCK";
    uint64_t now=time_us_64();
    if(now<sample_us||now-sample_us>C07_MAX_SAMPLE_AGE_US)return "STALE_SAMPLE";
    const c07_sample *s=&b->inputs.sample;
    c07_commutation_input input={.vin=s->vin,.vout=s->vout,.il=s->il,
        .duty=duty,.charging=charging,.now_us=now};
    c07_gate_plan plan;
    const char *error=c07_commutation_plan(&b->commutation,&c07_commissioning.commutation,&input,&plan);
    if(error)return error;
    if(time_us_64()-now>C07_PLAN_BUDGET_US)return "COMMUTATION_DEADLINE";
    error=c07_pwm_engine_submit(&b->pwm,plan.high_ticks,plan.low_ticks,
        now+C07_PIO_LEASE_US+C07_PLAN_BUDGET_US);
    if(error){b->pwm_rejected++;return error;}
    b->pwm_enabled=true;b->last_plan=plan;b->pwm_accepted++;
    b->last_plan_submitted_us=time_us_64();
    return NULL;
}
const char *c07_board_apply(c07_board *b, const c07_command *o) {
    const char *reason=NULL;
    if(o->bias_enable||o->pv_enable||o->battery_enable||o->pv_rearm)
        reason=b->configuration_error;
    if (!isfinite(o->duty)|| (o->duty!=0&&(o->duty<C07_DUTY_MIN||o->duty>C07_DUTY_MAX))) reason="PWM_DUTY";
    if (!reason&&o->pv_rearm&&(!o->pv_enable||o->duty!=0)) reason="REARM_COMMAND";
    if (!reason&&o->duty!=0&&(!o->bias_enable||!o->pv_enable)) reason="PWM_PERMISSION";
    status(b,true);
    const bool all_off=o->duty==0&&!o->bias_enable&&!o->pv_enable
        &&!o->battery_enable&&!o->pv_rearm;
    if(!reason&&all_off){
        bool was_bias_enabled=b->bias_enabled;
        c07_board_stop(b); /* sample/stop/retain causes before disabling BIAS */
        uint32_t events=b->inputs.pwm_events;
        if(b->inputs.battery_fault)return "BATTERY_INTERLOCK";
        if(b->inputs.pv_fault)return "PV_INTERLOCK";
        if(was_bias_enabled&&!b->inputs.bias_good)return "BIAS_INTERLOCK";
        if(events==C07_PWM_EVENT_PV_ONLY&&!b->inputs.pv_fault&&!b->inputs.battery_fault
           &&(!was_bias_enabled||b->inputs.bias_good)){
            /* The supervisor selected WAIT_INPUT; acknowledge only this exact
             * stopped-output case. Any hard, lease or unknown event is a fault. */
            b->inputs.pwm_events=0;return NULL;
        }
        return events?"PIO_INTERLOCK":NULL;
    }
    if (!reason&&(o->bias_enable||o->pv_enable||o->pv_rearm||o->battery_enable)) {
        uint64_t now=time_us_64();
        if (!b->inputs.sample.valid||o->sample_us!=b->inputs.sample_us||now<o->sample_us
            ||now-o->sample_us>C07_MAX_SAMPLE_AGE_US) reason="STALE_INPUTS";
    }
    if (!reason&&o->pv_enable&&b->inputs.pv_fault) reason="PV_INTERLOCK";
    if (!reason&&o->battery_enable&&b->inputs.battery_fault) reason="BATTERY_INTERLOCK";
    if (!reason&&o->duty!=0&&(!b->inputs.bias_good||b->inputs.pv_fault||b->inputs.pv_stopped)) reason="OUTPUT_INTERLOCK";
    bool rearm=c07_rearm_level(&b->rearm,o->pv_rearm,o->pv_enable,time_us_64());
    if (!reason&&rearm&&o->duty!=0) reason="REARM_ACTIVE";
    if (reason) { c07_board_stop(b); return reason; }
    bool removing=(b->bias_enabled&&!o->bias_enable)||(b->pv_enabled&&!o->pv_enable)
                    ||(b->battery_enabled&&!o->battery_enable);
    if ((o->duty==0||removing)&&pwm_off(b)) {
        c07_board_stop(b); return "PIO_INTERLOCK";
    }
    /* Check the PIO's retained event before enabling any power path, even when
     * PG has recovered. The later PWM check still catches newly arriving events. */
    if(pending_pio_fault(b)) { c07_board_stop(b); return "PIO_INTERLOCK"; }
    gpio_put(C07_BIAS_EN,o->bias_enable); gpio_put(C07_PV_RUN,o->pv_enable);
    gpio_put(C07_BAT_EN,o->battery_enable); gpio_put(C07_REARM,rearm);
    /* Taking this timestamp after the write makes the minimum hold conservative. */
    c07_rearm_written(&b->rearm,rearm,time_us_64());
    b->bias_enabled=o->bias_enable; b->pv_enabled=o->pv_enable; b->battery_enabled=o->battery_enable;
    reason=pwm_apply(b,o->duty,o->sample_us,o->synchronous_allowed);
    if (reason) c07_board_stop(b);
    return reason;
}
