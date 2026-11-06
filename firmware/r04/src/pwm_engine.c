#include "pwm_engine.h"
#include "c07_pwm.pio.h"
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
        pio_sm_set_enabled(e->pio,INTERLOCK_SM,false);
        pio_sm_set_enabled(e->pio,LEASE_SM,false);
        events |= c07_pwm_engine_events(e);
        pio_sm_init(e->pio,PWM_SM,e->pwm_offset,&e->pwm_config);
        pio_sm_init(e->pio,INTERLOCK_SM,e->interlock_offset,&e->interlock_config);
        pio_sm_init(e->pio,LEASE_SM,e->lease_offset,&e->lease_config);
        /* SM_RESTART in init also clears sticky pin-write masks. Only this
         * stopped-output path may release an asserted hardware clamp. */
        pio_sm_set_consecutive_pindirs(e->pio,PWM_SM,C07_PWM_H,2,true);
        pio_sm_set_pins_with_mask(e->pio,PWM_SM,0,(1u<<C07_PWM_H)|(1u<<C07_PWM_L));
        /* Preserve all bits above before W1C. Unknown bits remain caller faults. */
        e->pio->irq=0xffu;
        pio_sm_exec(e->pio,INTERLOCK_SM,pio_encode_set(pio_y,3));
    }
    e->enabled=false;e->lease_deadline_us=0;
    return events;
}
int c07_pwm_engine_init(c07_pwm_engine *e){
    memset(e,0,sizeof(*e));e->pio=pio0;pins_off();
    /* Reserve the WHOLE block: a higher-numbered SM must never override clamps. */
    for(unsigned sm=0;sm<4;sm++)if(pio_sm_is_claimed(e->pio,sm))return -1;
    if(!pio_can_add_program_at_offset(e->pio,&c07_pwm_program,0)
       ||!pio_can_add_program_at_offset(e->pio,&c07_interlock_program,16)
       ||!pio_can_add_program_at_offset(e->pio,&c07_lease_program,28))return -2;
    for(unsigned sm=0;sm<4;sm++)pio_sm_claim(e->pio,sm);
    e->pwm_offset=pio_add_program_at_offset(e->pio,&c07_pwm_program,0);
    e->interlock_offset=pio_add_program_at_offset(e->pio,&c07_interlock_program,16);
    e->lease_offset=pio_add_program_at_offset(e->pio,&c07_lease_program,28);
    e->pwm_config=c07_pwm_program_get_default_config(e->pwm_offset);
    sm_config_set_sideset_pins(&e->pwm_config,C07_PWM_H);
    sm_config_set_out_shift(&e->pwm_config,true,false,32);
    e->interlock_config=c07_interlock_program_get_default_config(e->interlock_offset);
    sm_config_set_set_pins(&e->interlock_config,C07_PWM_H,2);
    sm_config_set_out_shift(&e->interlock_config,true,false,32);
    sm_config_set_in_pins(&e->interlock_config,0);
    sm_config_set_out_special(&e->interlock_config,true,false,0);
    e->lease_config=c07_lease_program_get_default_config(e->lease_offset);
    sm_config_set_set_pins(&e->lease_config,C07_PWM_H,2);
    sm_config_set_out_shift(&e->lease_config,true,false,32);
    /* After SET0, pin writes remain asserted even during forced PULL/OUT X.
     * No normal refresh clears OUT_STICKY, restarts a SM, or clears IRQs. */
    sm_config_set_out_special(&e->lease_config,true,false,0);
    e->initialized=true;c07_pwm_engine_stop(e);return 0;
}
const char *c07_pwm_engine_submit(c07_pwm_engine *e,unsigned high,unsigned low,uint64_t valid_until_us){
    if(!e->initialized)return "PWM_NOT_INITIALIZED";
    if(c07_pwm_engine_fault(e))return "PIO_INTERLOCK";
    /* Sync: H + 128 + L + 131. Diode: H + OFF phase + 133.
     * The complete command fits one FIFO word, including the mode bit. */
    if(high<1||high>4096u||high+133u>=C07_PWM_TICKS||low==1||low>4096u
       ||(low&&high+low+259u!=C07_PWM_TICKS))return "PWM_COUNTS";
    unsigned phase=low?low:C07_PWM_TICKS-high-133u;
    uint32_t command=(high-1u)|(low?1u<<12:0u)|((phase-1u)<<13);
    uint32_t saved=save_and_disable_interrupts();
    uint64_t now=time_us_64();
    if(c07_pwm_engine_fault(e)){
        restore_interrupts(saved);return "PIO_INTERLOCK";
    }
    if(e->enabled&&now>=e->lease_deadline_us){
        restore_interrupts(saved);return "PIO_LEASE_EXPIRED";
    }
    /* Do not renew a control command which the PWM SM has not consumed.
     * At 50 kHz the previous one-word command normally left the FIFO long
     * before this 601 us control update. No queue of old commands/leases. */
    if(!pio_sm_is_tx_fifo_empty(e->pio,PWM_SM)
       ||!pio_sm_is_tx_fifo_empty(e->pio,LEASE_SM)){
        restore_interrupts(saved);return "PWM_BACKPRESSURE";
    }
    /* Anchor the expiry to the plan's clock, not a later submit timestamp.
     * All calculations above consume the same finite budget. */
    uint64_t lease_start=time_us_64();
    const uint32_t setup_guard_us=16;
    /* One microsecond covers the running loop's remaining <=31 clock
     * cycles, two forced instructions and expiry flag/SET instructions.
     * This is timer quantisation, not another electrical operating limit. */
    const uint32_t timer_margin_us=1;
    if(valid_until_us<=lease_start+setup_guard_us+timer_margin_us
       ||valid_until_us-lease_start>C07_PIO_LEASE_US+C07_PLAN_BUDGET_US){
        restore_interrupts(saved);return "PWM_PLAN_EXPIRED";
    }
    uint32_t remaining=(uint32_t)(valid_until_us-lease_start-setup_guard_us-timer_margin_us);
    /* Queue the new command BEFORE refreshing its lease. A CPU stop between
     * these writes leaves the old watchdog deadline, not an indefinite PWM. */
    pio_sm_put(e->pio,PWM_SM,command);
    pio_sm_put(e->pio,LEASE_SM,(remaining*(C07_CPU_HZ/1000000u))/32u-1u);
    /* Both instructions are nonblocking. If the CPU stops after PULL, the
     * old X keeps counting. OUT changes X only, never PC/IRQ or sticky pins.
     * Once JMP has fallen through, even a concurrent OUT cannot revive it. */
    pio_sm_exec(e->pio,LEASE_SM,pio_encode_pull(false,false));
    pio_sm_exec(e->pio,LEASE_SM,pio_encode_out(pio_x,32));
    if(pio_sm_is_exec_stalled(e->pio,LEASE_SM)){
        restore_interrupts(saved);return "PWM_LEASE_PRELOAD";
    }
    e->lease_deadline_us=valid_until_us-setup_guard_us;
    if(c07_pwm_engine_fault(e)){
        restore_interrupts(saved);return "PIO_INTERLOCK";
    }
    if(time_us_64()-lease_start>=setup_guard_us){
        restore_interrupts(saved);return "PWM_LEASE_SETUP";
    }
    if(!e->enabled){
        /* Only a completed stop/start changes the mux or enables the SMs. */
        pio_sm_set_enabled(e->pio,LEASE_SM,true);
        pio_sm_set_enabled(e->pio,INTERLOCK_SM,true);
        e->enabled=true;
        pio_gpio_init(e->pio,C07_PWM_H);pio_gpio_init(e->pio,C07_PWM_L);
        pio_sm_set_enabled(e->pio,PWM_SM,true);
    }
    if(c07_pwm_engine_fault(e)){
        restore_interrupts(saved);return "PIO_INTERLOCK";
    }
    restore_interrupts(saved);return NULL;
}
