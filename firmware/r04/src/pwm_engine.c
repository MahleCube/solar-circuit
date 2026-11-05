#include "pwm_engine.h"
#include "c07_pwm.pio.h"
#include "hardware/dma.h"
#include "hardware/regs/dma.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/time.h"
#include <string.h>

enum { PWM_SM=0, INTERLOCK_SM=1, LEASE_SM=2 };
static void pins_off(void){
    gpio_put(C07_PWM_H,0); gpio_put(C07_PWM_L,0);
    gpio_set_function(C07_PWM_H,GPIO_FUNC_SIO);gpio_set_function(C07_PWM_L,GPIO_FUNC_SIO);
    gpio_set_dir(C07_PWM_H,GPIO_OUT);gpio_set_dir(C07_PWM_L,GPIO_OUT);
}
uint32_t c07_pwm_engine_events(const c07_pwm_engine *e){
    if(!e->initialized)return 0;
    /* Cause tags precede IRQ0 by one PIO clock. Retain both volatile reads;
     * unknown or partial tags are not silently promoted to a PV-only event. */
    uint32_t events=e->pio->irq;
    return events|e->pio->irq;
}
bool c07_pwm_engine_fault(const c07_pwm_engine *e){
    return c07_pwm_engine_events(e)!=0;
}
uint32_t c07_pwm_engine_stop(c07_pwm_engine *e){
    pins_off();
    uint32_t events=c07_pwm_engine_events(e);
    if(e->initialized){
        /* The output mux is already SIO LOW. Disable producer before clamps. */
        pio_sm_set_enabled(e->pio,PWM_SM,false);
        dma_channel_abort(e->dma);
        /* A completed board stop is the sole DMA reset boundary. Error bits
         * are W1C; no partially transferred command survives this reset. */
        dma_hw->ch[e->dma].ctrl_trig=DMA_CH0_CTRL_TRIG_READ_ERROR_BITS|DMA_CH0_CTRL_TRIG_WRITE_ERROR_BITS;
        dma_hw->ch[e->dma].transfer_count=0;
        pio_sm_set_enabled(e->pio,INTERLOCK_SM,false);
        pio_sm_set_enabled(e->pio,LEASE_SM,false);
        events |= c07_pwm_engine_events(e);
        pio_sm_init(e->pio,PWM_SM,e->pwm_offset,&e->pwm_config);
        pio_sm_init(e->pio,INTERLOCK_SM,e->interlock_offset,&e->interlock_config);
        pio_sm_init(e->pio,LEASE_SM,e->lease_offset,&e->lease_config);
        pio_sm_set_consecutive_pindirs(e->pio,PWM_SM,C07_PWM_H,2,true);
        pio_sm_set_pins_with_mask(e->pio,PWM_SM,0,(1u<<C07_PWM_H)|(1u<<C07_PWM_L));
        /* Preserve all bits above before W1C. Unknown bits remain caller faults. */
        e->pio->irq=0xffu;
    }
    e->enabled=false;e->lease_deadline_us=0;
    return events;
}
int c07_pwm_engine_init(c07_pwm_engine *e){
    memset(e,0,sizeof(*e));e->dma=-1;e->pio=pio0;pins_off();
    /* Reserve the WHOLE block: a higher-numbered SM must never override clamps. */
    for(unsigned sm=0;sm<4;sm++)if(pio_sm_is_claimed(e->pio,sm))return -1;
    if(!pio_can_add_program_at_offset(e->pio,&c07_pwm_program,0)
       ||!pio_can_add_program_at_offset(e->pio,&c07_interlock_program,15)
       ||!pio_can_add_program_at_offset(e->pio,&c07_lease_program,28))return -2;
    e->dma=dma_claim_unused_channel(false);if(e->dma<0)return -3;
    for(unsigned sm=0;sm<4;sm++)pio_sm_claim(e->pio,sm);
    e->pwm_offset=pio_add_program_at_offset(e->pio,&c07_pwm_program,0);
    e->interlock_offset=pio_add_program_at_offset(e->pio,&c07_interlock_program,15);
    e->lease_offset=pio_add_program_at_offset(e->pio,&c07_lease_program,28);
    e->pwm_config=c07_pwm_program_get_default_config(e->pwm_offset);
    sm_config_set_sideset_pins(&e->pwm_config,C07_PWM_H);
    sm_config_set_out_shift(&e->pwm_config,true,false,32);
    e->interlock_config=c07_interlock_program_get_default_config(e->interlock_offset);
    sm_config_set_set_pins(&e->interlock_config,C07_PWM_H,2);
    sm_config_set_out_shift(&e->interlock_config,true,false,32);
    sm_config_set_in_pins(&e->interlock_config,0);
    e->lease_config=c07_lease_program_get_default_config(e->lease_offset);
    sm_config_set_set_pins(&e->lease_config,C07_PWM_H,2);
    sm_config_set_out_shift(&e->lease_config,true,false,32);
    e->initialized=true;c07_pwm_engine_stop(e);return 0;
}
const char *c07_pwm_engine_submit(c07_pwm_engine *e,unsigned high,unsigned low,uint64_t valid_until_us){
    if(!e->initialized)return "PWM_NOT_INITIALIZED";
    if(c07_pwm_engine_fault(e))return "PIO_INTERLOCK";
    if(dma_channel_is_busy(e->dma))return "PWM_BACKPRESSURE";
    if(e->enabled&&(dma_hw->ch[e->dma].transfer_count!=0
       ||(dma_hw->ch[e->dma].ctrl_trig&DMA_CH0_CTRL_TRIG_AHB_ERROR_BITS)))return "PWM_DMA_ERROR";
    /* 4 fetch/WAIT + 125 DT_HL + 2 OUT/JMP + optional 1 WAIT/125 DT_LH
     * + 1 OUT + tail. HS and LS loops each last exactly their tick count. */
    unsigned overhead=low?258u:132u;
    if(high<1||high>C07_PWM_TICKS||low==1||low>4096
       ||high+low+overhead>=C07_PWM_TICKS)return "PWM_COUNTS";
    unsigned tail=C07_PWM_TICKS-high-low-overhead;
    uint32_t word2=((tail-1u)<<12)|(low?low-1u:0u);
    for(unsigned k=0;k<C07_BURST_CYCLES;k++){
        e->words[2*k]=high-1u;e->words[2*k+1]=word2;
    }
    uint32_t saved=save_and_disable_interrupts();
    uint64_t now=time_us_64();
    if(c07_pwm_engine_fault(e)){
        restore_interrupts(saved);return "PIO_INTERLOCK";
    }
    if(e->enabled&&now>=e->lease_deadline_us){
        restore_interrupts(saved);return "PIO_LEASE_EXPIRED";
    }
    /* Updating a lease while SM0 is running races the autonomous clamp.
     * Take the pad to SIO LOW FIRST; then stop/reset the producer, and only
     * then renew the clamp. A late IRQ is observed while the pad remains LOW.
     * A fresh packet always starts on a two-word boundary, even after DMA error.
     * A previous final cycle may be shortened. This and the inter-packet gap
     * belong in the commissioned signed-current operating envelope. */
    pins_off();pio_sm_set_enabled(e->pio,PWM_SM,false);
    pio_sm_init(e->pio,PWM_SM,e->pwm_offset,&e->pwm_config);
    pio_sm_set_enabled(e->pio,LEASE_SM,false);
    if(c07_pwm_engine_fault(e)){
        restore_interrupts(saved);return "PIO_INTERLOCK";
    }
    pio_sm_init(e->pio,LEASE_SM,e->lease_offset,&e->lease_config);
    /* Anchor the expiry to the plan's clock, not a later submit timestamp.
     * All calculations above and DMA stalls consume the same finite budget. */
    uint64_t lease_start=time_us_64();
    const uint32_t setup_guard_us=16;
    if(valid_until_us<=lease_start+setup_guard_us
       ||valid_until_us-lease_start>C07_PIO_LEASE_US+C07_PLAN_BUDGET_US){
        restore_interrupts(saved);return "PWM_PLAN_EXPIRED";
    }
    uint32_t remaining=(uint32_t)(valid_until_us-lease_start-setup_guard_us);
    pio_sm_put(e->pio,LEASE_SM,(remaining*(C07_CPU_HZ/1000000u))/32u-1u);
    /* SM2 remains disabled: preload X without spending instruction memory on
     * PULL/OUT in the running lease. FIFO has just been reset and one word put. */
    pio_sm_exec(e->pio,LEASE_SM,pio_encode_pull(false,true));
    pio_sm_exec(e->pio,LEASE_SM,pio_encode_out(pio_x,32));
    if(pio_sm_is_exec_stalled(e->pio,LEASE_SM)){
        restore_interrupts(saved);return "PWM_LEASE_PRELOAD";
    }
    e->lease_deadline_us=valid_until_us-setup_guard_us;
    pio_sm_set_enabled(e->pio,LEASE_SM,true);
    if(c07_pwm_engine_fault(e)){
        restore_interrupts(saved);return "PIO_INTERLOCK";
    }
    if(time_us_64()-lease_start>=setup_guard_us){
        restore_interrupts(saved);return "PWM_LEASE_SETUP";
    }
    if(!e->enabled){
        pio_sm_set_enabled(e->pio,INTERLOCK_SM,true);
        e->enabled=true;
    }
    /* Explicit inter-packet both-OFF interval; the GPIO-to-channel delays
     * remain separately constrained by the commissioning profile. */
    busy_wait_us_32(2);
    pio_gpio_init(e->pio,C07_PWM_H);pio_gpio_init(e->pio,C07_PWM_L);
    pio_sm_set_enabled(e->pio,PWM_SM,true);
    dma_channel_config config=dma_channel_get_default_config(e->dma);
    channel_config_set_transfer_data_size(&config,DMA_SIZE_32);
    channel_config_set_read_increment(&config,true);channel_config_set_write_increment(&config,false);
    channel_config_set_dreq(&config,pio_get_dreq(e->pio,PWM_SM,true));
    dma_channel_configure(e->dma,&config,&e->pio->txf[PWM_SM],e->words,2*C07_BURST_CYCLES,true);
    restore_interrupts(saved);return NULL;
}
