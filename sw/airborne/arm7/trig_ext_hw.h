#ifndef TRIG_EXT_HW_H
#define TRIG_EXT_HW_H


/**
 *  falling/rising edge
 */
#define TRIG_EXT_EDGE_RISING 1
#define TRIG_EXT_EDGE_FALLING 0



#include "LPC21xx.h"
#include BOARD_CONFIG


static inline void trig_ext_init ( void ) {
  /* select pin for capture */
  PPM_PINSEL |= PPM_PINSEL_VAL << PPM_PINSEL_BIT;
  /* enable capture 0.2 on falling or rising edge + trigger interrupt */
#if defined TRIG_EXT_PULSE_TYPE && TRIG_EXT_PULSE_TYPE == TRIG_EXT_PULSE_TYPE_RISING
  T0CCR = PPM_CCR_CRF | PPM_CCR_CRI;
#elif defined TRIG_EXT_PULSE_TYPE && TRIG_EXT_PULSE_TYPE == TRIG_EXT_PULSE_TYPE_FALLING
  T0CCR = PPM_CCR_CRR | PPM_CCR_CRI;
#else
#error "trig_ext_hw.h: Unknown PULSE_TYPE"
#endif
  trig_ext_valid = FALSE;
}

#define TRIG_ISR() {						\
    static uint32_t last;					\
    trigger_t0 = PPM_CR;					\
    delta_t0 = trigger_t0 - last;				\
    last = trigger_t0;						\
    trig_ext_valid = TRUE;					\
}


#endif /* TRIG_EXT_HW_H */

