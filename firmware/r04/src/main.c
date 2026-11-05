#include "board.h"
#include "board_config.h"
#include "command.h"
#include "selection_config.h"
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/watchdog.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/structs/timer.h"
#include "hardware/address_mapped.h"
#define C07_IRQ_RETURN_GUARD_US 16u
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>

/* TIMER IRQ exclusively owns board, charger, SPI, PIO and watchdog.
 * Main owns USB parsing and background MPPT local computation. */
static c07_board board;
static c07_charger charger;
static c07_command_parser command_parser;
static _Atomic uint32_t requested_run,requested_reset,requested_status;
static uint32_t consumed_reset,served_status;
static bool watchdog_started;
static uint64_t next_tick_us,previous_frame,previous_sample;
static uint32_t max_loop_us,max_adc_us,max_control_us,max_lateness_us,overruns;
/* Diagnostic counters are integer stores only, with their cost included. */
volatile uint32_t c07_background_active,c07_background_jobs;
volatile uint64_t c07_scheduled_us,c07_irq_entry_us,c07_control_start_us,c07_irq_exit_us;

typedef struct {
 c07_state state;const char *fault;c07_sample sample;uint16_t raw[8];
 uint64_t frames,accepted,rejected;uint32_t loop_max,adc_max,control_max,lateness_max,overruns;
 uint32_t mppt_published,mppt_applied,mppt_rejected,mppt_expired;
} c07_telemetry;
static volatile c07_telemetry telemetry;
static _Atomic uint32_t telemetry_seq;
static uint32_t printed_status;

static void publish_status(uint32_t id){
 atomic_store_explicit(&telemetry_seq,id*2u-1u,memory_order_relaxed);
 atomic_thread_fence(memory_order_release);
 telemetry.state=charger.state;telemetry.fault=charger.fault;telemetry.sample=board.inputs.sample;
 for(unsigned k=0;k<8;k++)telemetry.raw[k]=board.raw[k];
 telemetry.frames=board.completed_frames;telemetry.accepted=board.pwm_accepted;telemetry.rejected=board.pwm_rejected;
 telemetry.loop_max=max_loop_us;telemetry.adc_max=max_adc_us;telemetry.control_max=max_control_us;
 telemetry.lateness_max=max_lateness_us;telemetry.overruns=overruns;
 telemetry.mppt_published=charger.regulator.async.published;telemetry.mppt_applied=charger.regulator.async.applied;
 telemetry.mppt_rejected=charger.regulator.async.rejected;telemetry.mppt_expired=charger.regulator.async.expired;
 atomic_store_explicit(&telemetry_seq,id*2u,memory_order_release);served_status=id;
}
static void control_timer_irq(void){
 timer_hw->intr=1u;
 uint64_t scheduled=next_tick_us,entry=time_us_64();next_tick_us=scheduled+C07_LOOP_US;
 timer_hw->alarm[0]=(uint32_t)next_tick_us;
 c07_scheduled_us=scheduled;c07_irq_entry_us=entry;
 uint32_t late=entry>scheduled?(uint32_t)(entry-scheduled):0;
 if(late>max_lateness_us)max_lateness_us=late;
 c07_request request={atomic_load_explicit(&requested_run,memory_order_acquire)!=0,false};
 uint32_t reset_id=atomic_load_explicit(&requested_reset,memory_order_acquire);
 request.reset_fault=reset_id!=consumed_reset;consumed_reset=reset_id;
 if(request.run&&!watchdog_started){watchdog_enable(C07_WATCHDOG_MS,true);watchdog_started=true;}
 if(entry>=next_tick_us){c07_board_stop(&board);c07_charger_trip(&charger,"CONTROL_DEADLINE",entry);goto deadline;}
 const c07_inputs *input=c07_board_read(&board);
 if(input->sample.valid){
  if(input->frame_id<=previous_frame||(previous_frame&&input->sample_us<=previous_sample)
    ||input->completed_us<input->sample_us||input->completed_us>time_us_64()){
   board.inputs.sample.valid=false;board.inputs.error="ADC_FRAME_ORDER";
  }else{previous_frame=input->frame_id;previous_sample=input->sample_us;}
 }
 if(input->age_us>max_adc_us)max_adc_us=input->age_us;
 uint64_t control_start=time_us_64();c07_control_start_us=control_start;
 const c07_command *output=c07_charger_step(&charger,input,&request,control_start);
 if(charger.state!=C07_CHARGING)c07_mppt_invalidate(&charger.regulator,true);
 uint32_t control_us=(uint32_t)(time_us_64()-control_start);
 if(control_us>max_control_us)max_control_us=control_us;
 if(time_us_64()>=next_tick_us){c07_board_stop(&board);c07_charger_trip(&charger,"CONTROL_DEADLINE",time_us_64());goto deadline;}
 const char *error=c07_board_apply(&board,output);
 if(error){
  /* An input stop can arrive after the supervisor sampled its frame.
   * board retains PIO causes and the GPIO fault snapshot before turning the
   * bias supply off. Only an identified ordinary PV stop is retryable. */
  c07_board_stop(&board);
  /* stop() can capture an additional fault while clamping the outputs.
   * Classify AFTER that snapshot; neither headroom nor a PV tag may erase it. */
  bool clean_stop=board.inputs.bias_good&&!board.inputs.pv_fault&&!board.inputs.battery_fault
      &&(!board.inputs.pwm_events||board.inputs.pwm_events==3u);
  bool input_stop=clean_stop&&(!strcmp(error,"PIO_INTERLOCK")||!strcmp(error,"OUTPUT_INTERLOCK"))
      &&(board.inputs.pwm_events==3u
         ||(!board.inputs.pwm_events&&board.inputs.pv_stopped));
  if((!strcmp(error,"INPUT_HEADROOM_LOST")&&clean_stop)||input_stop)
      c07_charger_wait_input(&charger,time_us_64());
  else {
   if(board.inputs.battery_fault)error="BATTERY_FAULT";
   else if(board.inputs.pv_fault)error="PV_OVERCURRENT";
   else if(board.inputs.pwm_events&4u)error="PIO_LEASE_EXPIRED";
   else if(board.inputs.pwm_events&&board.inputs.pwm_events!=3u)error="PIO_INTERLOCK";
   else if(!board.inputs.bias_good&&!strcmp(error,"INPUT_HEADROOM_LOST"))error="BIAS_LOST";
   c07_charger_trip(&charger,error,time_us_64());
  }
 }
 if(watchdog_started)watchdog_update();
 deadline:;
 uint32_t status_id=atomic_load_explicit(&requested_status,memory_order_acquire);
 if(status_id!=served_status&&!request.run&&(charger.state==C07_IDLE||charger.state==C07_FAULT))publish_status(status_id);
 uint64_t end=time_us_64();uint32_t elapsed=(uint32_t)(end-scheduled);
 if(elapsed>max_loop_us)max_loop_us=elapsed;
 /* Explicit budget for timestamp stores and IRQ epilogue. The fixture also
  * checks the actual exception-return against the absolute next deadline. */
 if(end+C07_IRQ_RETURN_GUARD_US>=next_tick_us){
  overruns++;c07_board_stop(&board);c07_charger_trip(&charger,"CONTROL_DEADLINE",end);
  /* A failed epoch never issues fictitious catch-up control calls. */
  timer_hw->intr=1u;irq_clear(TIMER_IRQ_0);next_tick_us=time_us_64()+C07_LOOP_US;
  timer_hw->alarm[0]=(uint32_t)next_tick_us;
 }
 c07_irq_exit_us=time_us_64();
}
static void poll_command(void){
 for(unsigned k=0;k<16;k++){
  int ch=getchar_timeout_us(0);if(ch==PICO_ERROR_TIMEOUT)return;
  switch(c07_command_feed(&command_parser,ch)){
  case C07_COMMAND_ARM:atomic_store_explicit(&requested_run,1,memory_order_release);break;
  case C07_COMMAND_DISARM:atomic_store_explicit(&requested_run,0,memory_order_release);break;
  case C07_COMMAND_RESET:if(!atomic_load(&requested_run))atomic_store(&requested_reset,atomic_load(&requested_reset)+1u);break;
  case C07_COMMAND_STATUS:atomic_store(&requested_status,atomic_load(&requested_status)+1u);break;
  case C07_COMMAND_NONE:break;
  }
 }
}
static void print_status(void){
 if(atomic_load_explicit(&requested_run,memory_order_acquire))return;
 uint32_t seq=atomic_load_explicit(&telemetry_seq,memory_order_acquire);
 if(!seq||(seq&1u)||seq==printed_status)return;
 static c07_telemetry t;t=telemetry;atomic_thread_fence(memory_order_acquire);
 if(seq!=atomic_load_explicit(&telemetry_seq,memory_order_acquire))return;
 printed_status=seq;
 printf("state=%s fault=%s frames=%llu adc_max_us=%lu control_max_us=%lu loop_max_us=%lu overruns=%lu accepted=%llu rejected=%llu\n",
  c07_state_name(t.state),t.fault,(unsigned long long)t.frames,(unsigned long)t.adc_max,(unsigned long)t.control_max,
  (unsigned long)t.loop_max,(unsigned long)t.overruns,(unsigned long long)t.accepted,(unsigned long long)t.rejected);
 printf("measurements valid=%u vin=%.6f ipv=%.6f vout=%.6f il=%.6f tbat=%.6f tsink=%.6f vbat=%.6f vpv=%.6f\n",
  (unsigned)t.sample.valid,(double)t.sample.vin,(double)t.sample.ipv,(double)t.sample.vout,(double)t.sample.il,
  (double)t.sample.tbat,(double)t.sample.tsink,(double)t.sample.vbat,(double)t.sample.vpv);
 printf("async published=%lu applied=%lu rejected=%lu expired=%lu irq_lateness_us=%lu\n",
  (unsigned long)t.mppt_published,(unsigned long)t.mppt_applied,(unsigned long)t.mppt_rejected,(unsigned long)t.mppt_expired,(unsigned long)t.lateness_max);
 printf("adc_raw_ch0_to_ch7=%u,%u,%u,%u,%u,%u,%u,%u\n",t.raw[0],t.raw[1],t.raw[2],t.raw[3],t.raw[4],t.raw[5],t.raw[6],t.raw[7]);
}
int main(void){
 unsigned safe_pins[]={6,7,12,10,11,8};
 for(unsigned k=0;k<6;k++){gpio_init(safe_pins[k]);gpio_put(safe_pins[k],0);gpio_set_dir(safe_pins[k],GPIO_OUT);}
 if(!set_sys_clock_khz(C07_CPU_HZ/1000,true))while(true)tight_loop_contents();
 static c07_config config;c07_config_defaults(&config);
 config.CONTROL_PERIOD_US=C07_LOOP_US;config.MAX_LOOP_GAP_US=C07_MAX_LOOP_GAP_US;
 config.DUTY_MIN=C07_DUTY_MIN;config.DUTY_MAX=C07_DUTY_MAX;
 config.CHARGE_CURRENT_A=C07_CHARGE_CURRENT_A;config.CHARGE_VOLTAGE_V=C07_CHARGE_VOLTAGE_V;
 config.VOUT_TRIP_V=C07_VOUT_TRIP_V;config.PV_HEADROOM_V=C07_HEADROOM_V;
 config.CONNECT_OFFSET_V=C07_CONNECT_OFFSET_V;config.RAMP_CURRENT_A=C07_RAMP_CURRENT_A;config.TSINK_MAX_C=C07_TSINK_MAX_C;
 config.RAMP_VOLTAGE_ERROR_V=C07_RAMP_VOLTAGE_ERROR_V;
 config.RAMP_PACKET_US=C07_PIO_LEASE_US+C07_PLAN_BUDGET_US;
 c07_selection_apply(&config);
 if(c07_board_init(&board)!=0||c07_charger_init(&charger,&config)!=0){c07_board_stop(&board);while(true)tight_loop_contents();}
 stdio_init_all();atomic_store(&requested_run,C07_ARM_ON_BOOT&&!watchdog_caused_reboot());
 printf("c07 r04: DISARMED; synchronous commutation, independent PIO interlocks, background IncCond.\n");
 hardware_alarm_claim(0);irq_set_exclusive_handler(TIMER_IRQ_0,control_timer_irq);irq_set_priority(TIMER_IRQ_0,0);
 next_tick_us=time_us_64()+C07_LOOP_US;timer_hw->intr=1u;hw_set_bits(&timer_hw->inte,1u);timer_hw->alarm[0]=(uint32_t)next_tick_us;
 irq_set_enabled(TIMER_IRQ_0,true);
 while(true){
  static c07_mppt_job job;static c07_mppt_result result;
  if(c07_mppt_begin(&charger.regulator,&job)){
#if C07_TEST_BACKGROUND_START_OFFSET_US > 0
   /* Diagnostic build only: make genuine MPPT arithmetic overlap a timer IRQ. */
   busy_wait_until(from_us_since_boot(c07_scheduled_us+C07_TEST_BACKGROUND_START_OFFSET_US));
#endif
   c07_background_active=1;c07_mppt_compute(&charger.regulator.cfg,&job,&result);
   c07_background_active=2;c07_mppt_publish(&charger.regulator,&result);c07_background_active=0;c07_background_jobs++;
  }
  poll_command();print_status();tight_loop_contents();
 }
}
