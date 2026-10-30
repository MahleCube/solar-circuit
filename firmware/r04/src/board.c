#include "board.h"
#include "board_config.h"
#include "adc_convert.h"
#include "c07_pwm.pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/time.h"
#include <math.h>
#include <string.h>

/* Per-channel physical-unit calibration: measured_nominal*gain+offset. */
static const c07_real_t cal_gain[8]={1,1,1,1,1,1,1,1};
static const c07_real_t cal_offset[8]={0,0,0,0,0,0,0,0};
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
    bool fault=false;
    if(b->initialized) {
        while(!pio_sm_is_rx_fifo_empty(b->pio,b->sm))
            if(pio_sm_get(b->pio,b->sm)!=0) fault=true;
    }
    return fault;
}
static bool pwm_off(c07_board *b) {
    bool fault=false;
    output_low(C07_PWM_H); output_low(C07_PWM_L);
    if (b->initialized) {
        pio_sm_set_enabled(b->pio,b->sm,false);
        /* Preserve a latched PG-loss event even if this command requests zero. */
        fault=pending_pio_fault(b);
        pio_sm_clear_fifos(b->pio,b->sm);
        pio_sm_restart(b->pio,b->sm);
        pio_sm_init(b->pio,b->sm,b->offset,&b->pwm_config);
        pio_sm_set_consecutive_pindirs(b->pio,b->sm,C07_PWM_H,2,true);
        pio_sm_set_pins_with_mask(b->pio,b->sm,0,(1u<<C07_PWM_H)|(1u<<C07_PWM_L));
    }
    b->pwm_enabled=false;
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
    for(unsigned n=0;n<sizeof(output_pins)/sizeof(output_pins[0]);n++) output_low(output_pins[n]);
    if (clock_get_hz(clk_sys)!=C07_CPU_HZ) return -1;
    unsigned inputs[]={C07_BIAS_PG,C07_BAT_FLT_N,C07_PV_FLT_N,C07_PV_STOPPED};
    for(unsigned n=0;n<4;n++) { gpio_init(inputs[n]); gpio_set_dir(inputs[n],GPIO_IN); }
    gpio_pull_up(C07_BIAS_PG); gpio_pull_up(C07_BAT_FLT_N); gpio_pull_up(C07_PV_FLT_N);
    /* PV_STOPPED polarity/pull is owned by the circuit, matching the saved contract. */
    gpio_disable_pulls(C07_PV_STOPPED);
    b->pio=pio0; b->sm=0;
    if (pio_sm_is_claimed(b->pio,b->sm) || !pio_can_add_program(b->pio,&c07_pwm_program)) return -2;
    pio_sm_claim(b->pio,b->sm);
    b->offset=pio_add_program(b->pio,&c07_pwm_program);
    b->pwm_config=c07_pwm_program_get_default_config(b->offset);
    sm_config_set_set_pins(&b->pwm_config,C07_PWM_H,2);
    sm_config_set_jmp_pin(&b->pwm_config,C07_BIAS_PG);
    sm_config_set_out_shift(&b->pwm_config,true,false,32);
    sm_config_set_clkdiv(&b->pwm_config,1.0);
    b->initialized=true; pwm_off(b);
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
static const char *pwm_apply(c07_board *b, c07_real_t duty, uint64_t sample_us) {
    if (duty==0) return pwm_off(b) ? "PIO_BIAS_LOST" : NULL;
    if (!isfinite(duty)||duty<C07_DUTY_MIN||duty>C07_DUTY_MAX) return "PWM_DUTY";
    if(pending_pio_fault(b)) return "PIO_BIAS_LOST";
    uint64_t now=time_us_64();
    if (now<sample_us||now-sample_us>C07_MAX_SAMPLE_AGE_US) return "STALE_SAMPLE";
    if (!pio_sm_is_tx_fifo_empty(b->pio,b->sm)) { b->pwm_rejected++; return "PWM_BACKPRESSURE"; }
    unsigned high=(unsigned)(duty*C07_PWM_TICKS),low=C07_PWM_TICKS-high;
    if (high<2||high>4097||low<8||low>16391) return "PWM_COUNTS";
    uint32_t word=(C07_BURST_CYCLES-1)|((high-2)<<6)|((low-8)<<18);
    if (!b->pwm_enabled) {
        pio_gpio_init(b->pio,C07_PWM_H); pio_gpio_init(b->pio,C07_PWM_L);
        pio_sm_set_enabled(b->pio,b->sm,true); b->pwm_enabled=true;
    }
    now=time_us_64();
    if (now<sample_us||now-sample_us>C07_MAX_SAMPLE_AGE_US) return "STALE_SAMPLE";
    pio_sm_put(b->pio,b->sm,word); /* FIFO was empty; nonblocking write. */
    b->pwm_accepted++;
    return NULL;
}
const char *c07_board_apply(c07_board *b, const c07_command *o) {
    const char *reason=NULL;
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
        c07_board_stop(b); return "PIO_BIAS_LOST";
    }
    /* Check the PIO's retained event before enabling any power path, even when
     * PG has recovered. The later PWM check still catches newly arriving events. */
    if(pending_pio_fault(b)) { c07_board_stop(b); return "PIO_BIAS_LOST"; }
    gpio_put(C07_BIAS_EN,o->bias_enable); gpio_put(C07_PV_RUN,o->pv_enable);
    gpio_put(C07_BAT_EN,o->battery_enable); gpio_put(C07_REARM,rearm);
    /* Taking this timestamp after the write makes the minimum hold conservative. */
    c07_rearm_written(&b->rearm,rearm,time_us_64());
    b->bias_enabled=o->bias_enable; b->pv_enabled=o->pv_enable; b->battery_enabled=o->battery_enable;
    reason=pwm_apply(b,o->duty,o->sample_us);
    if (reason) c07_board_stop(b);
    return reason;
}
