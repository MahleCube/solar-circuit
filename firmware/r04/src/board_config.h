#ifndef C07_BOARD_CONFIG_H
#define C07_BOARD_CONFIG_H
#include <stdint.h>

/* Saved c07 wiring; these are RP2040 GPIO numbers, not connector pin numbers. */
enum { C07_PWM_H=6, C07_PWM_L=7, C07_BIAS_EN=8, C07_BIAS_PG=9,
       C07_PV_RUN=10, C07_BAT_EN=11, C07_REARM=12, C07_BAT_FLT_N=13,
       C07_PV_FLT_N=14, C07_PV_STOPPED=15,
       C07_SPI_MISO=16, C07_SPI_CS=17, C07_SPI_SCK=18, C07_SPI_MOSI=19 };
#define C07_CPU_HZ 125000000u
#define C07_PWM_HZ 50000u
#define C07_PWM_TICKS 2500u
#define C07_BURST_CYCLES 50u
/* 50 PWM periods plus 5 PIO packet instructions = 1000.040 us at 125 MHz.
 * Round up the scheduler interval so equal-period packets cannot accumulate. */
#define C07_LOOP_US 1001u
#define C07_MAX_LOOP_GAP_US 5000u
#define C07_MAX_SAMPLE_AGE_US 2000u
#define C07_SPI_HZ 1000000u
#define C07_WATCHDOG_MS 1000u
#define C07_DUTY_MIN 0.03
#define C07_DUTY_MAX 0.90
#ifndef C07_ARM_ON_BOOT
#define C07_ARM_ON_BOOT 0
#endif

/* Nominal calibration and initial limits, not measured/qualified battery values. */
#define C07_ADC_REFERENCE_V 2.5
#define C07_DIVIDER_RATIO (153300.0/3300.0)
#define C07_CHARGE_CURRENT_A 15.0
#define C07_CHARGE_VOLTAGE_V 57.1
/* 15 A is an operating current target, not a single-sample fault boundary.
 * Do not invent a replacement peak-current trip from a thermal nameplate.
 * ADC rail validation and the independent PV/BAT hardware fault inputs remain.
 * Neither the 15 A target nor those remote shunts guarantee an L1 peak bound.
 */
#define C07_REVERSE_TRIP_A (-0.5)
#define C07_VIN_TRIP_V 105.0
#define C07_VPV_TRIP_V 105.0
#define C07_VOUT_TRIP_V 58.0
#define C07_VBAT_MIN_V 42.0
#define C07_VBAT_TRIP_V 58.0
#define C07_TBAT_MIN_C 0.0
#define C07_TBAT_MAX_C 45.0
#define C07_TSINK_MAX_C 80.0
#define C07_HEADROOM_V 5.0

#define C07_CLEAR_US 20000u
#define C07_REARM_US 2000u
#define C07_PV_TIMEOUT_US 2000000u
#define C07_INPUT_STABLE_US 20000u
#define C07_BIAS_TIMEOUT_US 2000000u
#define C07_BIAS_STABLE_US 20000u
#define C07_RAMP_TIMEOUT_US 5000000u
#define C07_CONNECT_STABLE_US 20000u
#define C07_CONNECT_TIMEOUT_US 500000u
#define C07_CONNECT_DELTA_V 0.3
#define C07_CONNECT_OFFSET_V 0.1
#define C07_RUN_DELTA_V 1.0
#define C07_RAMP_CURRENT_A 0.5
#define C07_STOP_CURRENT_A 0.3
#define C07_STOP_STABLE_US 10000u
#define C07_STOP_TIMEOUT_US 100000u
#endif
