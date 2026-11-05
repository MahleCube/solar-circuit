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
static const uint8_t adc_order[8]={0,7,1,3,2,6,4,5};
static const unsigned output_pins[]={C07_PWM_H,C07_PWM_L,C07_REARM,C07_PV_RUN,C07_BAT_EN,C07_BIAS_EN};

static void output_low(unsigned pin) {
    /* Set the SIO output latch before switching the pad away from PIO. */
    gpio_put(pin,0); gpio_set_function(pin,GPIO_FUNC_SIO); gpio_set_dir(pin,GPIO_OUT);
}
static void status(c07_board *b) {
    b->inputs.bias_good=!gpio_get(C07_BIAS_PG);
    b->inputs.battery_fault=!gpio_get(C07_BAT_FLT_N);
    b->inputs.pv_fault=!gpio_get(C07_PV_FLT_N);
    b->inputs.pv_stopped=gpio_get(C07_PV_STOPPED);
}
static bool pending_pio_fault(c07_board *b) {
    return c07_pwm_engine_fault(&b->pwm);
}
static bool pwm_off(c07_board *b) {
    bool fault=c07_pwm_engine_stop(&b->pwm);
    b->pwm_enabled=false;c07_commutation_reset(&b->commutation);
    return fault;
}
void c07_board_stop(c07_board *b) {
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
const c07_inputs *c07_board_read(c07_board *b) {
    c07_inputs *i=&b->inputs;
    i->sample.valid=false; i->error=NULL; i->completed_us=0;
    i->sample.pv_pair_span_us=-1;
    i->sample_us=time_us_64();
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
    }
    if (!i->error) {
        c07_real_t physical[8];
        for(unsigned ch=0;ch<8;ch++) {
            c07_real_t value=c07_adc_nominal(ch,b->raw[ch]);
            if (!isfinite(cal_gain[ch])||cal_gain[ch]<=0||!isfinite(cal_offset[ch])) {
                i->error="CAL_INVALID"; break;
            }
            physical[ch]=value*cal_gain[ch]+cal_offset[ch];
            if (!isfinite(physical[ch])) { i->error="ADC_NONFINITE"; break; }
        }
        if (!i->error) {
            i->sample.vin=physical[0]; i->sample.ipv=physical[1];
            i->sample.vout=physical[2]; i->sample.il=physical[3];
            i->sample.tbat=physical[4]; i->sample.tsink=physical[5];
            i->sample.vbat=physical[6]; i->sample.vpv=physical[7];
        }
    }
    status(b);
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
    return NULL;
}
const char *c07_board_apply(c07_board *b, const c07_command *o) {
    const char *reason=NULL;
    if(o->bias_enable||o->pv_enable||o->battery_enable||o->pv_rearm)
        reason=b->configuration_error;
    if(!reason&&o->battery_enable&&!b->battery_enabled){
        double d=b->inputs.sample.vout-b->inputs.sample.vbat;
        double e=c07_commissioning.commutation.vout_error_v+c07_commissioning.commutation.battery_voltage_error_v;
        if(d-e<0||d+e>C07_CONNECT_DELTA_V)reason="BATTERY_CONNECT_UNCERTAINTY";
    }
    if (!isfinite(o->duty)|| (o->duty!=0&&(o->duty<C07_DUTY_MIN||o->duty>C07_DUTY_MAX))) reason="PWM_DUTY";
    if (!reason&&o->pv_rearm&&(!o->pv_enable||o->duty!=0)) reason="REARM_COMMAND";
    if (!reason&&o->duty!=0&&(!o->bias_enable||!o->pv_enable)) reason="PWM_PERMISSION";
    status(b);
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
