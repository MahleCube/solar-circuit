#ifndef C07_PWM_ENGINE_H
#define C07_PWM_ENGINE_H
#include <stdbool.h>
#include <stdint.h>
#include "hardware/pio.h"
#include "board_config.h"
typedef struct {
    PIO pio;
    unsigned pwm_offset, interlock_offset, lease_offset;
    pio_sm_config pwm_config, interlock_config, lease_config;
    int dma;
    bool initialized, enabled;
    uint64_t lease_deadline_us;
    uint32_t words[2*C07_BURST_CYCLES];
} c07_pwm_engine;
enum { C07_PWM_EVENT_STOP=1u, C07_PWM_EVENT_PV_STOPPED=2u,
       C07_PWM_EVENT_LEASE=4u, C07_PWM_EVENT_PV_ONLY=3u };
int c07_pwm_engine_init(c07_pwm_engine *);
/* stop returns every observed IRQ bit before clearing the hardware latches.
 * Callers retain/classify the event; a cleared PIO bit is not fault permission. */
uint32_t c07_pwm_engine_stop(c07_pwm_engine *);
uint32_t c07_pwm_engine_events(const c07_pwm_engine *);
bool c07_pwm_engine_fault(const c07_pwm_engine *);
/* high/low are GPIO HIGH durations, in sysclk ticks. low==0 skips Q2.
 * Nominal no-stall cycle period is exactly C07_PWM_TICKS. */
const char *c07_pwm_engine_submit(c07_pwm_engine *,unsigned high,unsigned low,uint64_t valid_until_us);
#endif
