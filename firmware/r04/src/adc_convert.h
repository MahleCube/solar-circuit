#ifndef C07_ADC_CONVERT_H
#define C07_ADC_CONVERT_H
#include <stdint.h>
#include "c07_control.h"
#include "board_config.h"

/* Same nominal transfer functions as the prior board.c, with compile-time
 * affine coefficients. Keep channel-specific gain/offset calibration outside
 * this conversion. No acquisition/order/fault-monitoring changes. */
static inline c07_real_t c07_adc_nominal(unsigned channel,uint16_t raw){
    if(channel==1||channel==3)
        return (c07_real_t)raw*(C07_ADC_REFERENCE_V/4096.0/0.05)-(1.25/0.05);
    if(channel==4||channel==5)
        return (c07_real_t)raw*(C07_ADC_REFERENCE_V/4096.0/0.010)-(0.5/0.010);
    return (c07_real_t)raw*(C07_ADC_REFERENCE_V/4096.0*C07_DIVIDER_RATIO);
}
#endif
