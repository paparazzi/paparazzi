#ifndef ICP_SCALE_H
#define ICP_SCALE_H

/* INPUT CAPTURE  on P0.6*/
#define ICP_PINSEL     PINSEL0
#define ICP_PINSEL_VAL 0x02
#define ICP_PINSEL_BIT 12

#include "led.h"

volatile uint32_t pulse_len;

static inline void icp_scale_init(void)
{
  /* select pin for capture */
  ICP_PINSEL |= ICP_PINSEL_VAL << ICP_PINSEL_BIT;
  /* enable capture 0.2 on falling edge + trigger interrupt */
  T0CCR |= TCCR_CR2_F | TCCR_CR2_I;
}


#define ICP_ISR() {           \
    static uint32_t last;         \
    uint32_t now = T0CR2;         \
    pulse_len = now - last;         \
    last = now;             \
    LED_TOGGLE(2);            \
  }



#endif /* ICP_SCALE_H */
