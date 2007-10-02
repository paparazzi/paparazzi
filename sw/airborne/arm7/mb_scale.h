#ifndef MB_SCALE_H
#define MB_SCALE_H

/* P0.29 CAP0.3 */
#define ICP_PINSEL     PINSEL1
#define ICP_PINSEL_VAL 0x02
#define ICP_PINSEL_BIT 26

#include "led.h"

extern volatile uint32_t pulse_len;
extern float mb_scale_thrust;
extern float mb_scale_torque;

void mb_scale_init ( void );
void mb_scale_periodic(void);



#define MB_SCALE_ICP_ISR() {					\
    static uint32_t last;					\
    uint32_t now = T0CR3;					\
    pulse_len = now - last;					\
    last = now;							\
    LED_TOGGLE(2);						\
  }



#endif /* MB_SCALE_H */
