#ifndef MB_SCALE_H
#define MB_SCALE_H

#include "std.h"
#include "LPC21xx.h"

/* P0.29 CAP0.3 */
#define ICP_PINSEL     PINSEL1
#define ICP_PINSEL_VAL 0x02
#define ICP_PINSEL_BIT 26

extern volatile uint32_t mb_scale_pulse_len;
extern float mb_scale_thrust;
extern float mb_scale_torque;

extern uint32_t mb_scale_neutral;
extern float    mb_scale_gain;

void mb_scale_init ( void );


#define MB_SCALE_ICP_ISR() {					\
    static uint32_t last;					\
    uint32_t now = T0CR3;					\
    mb_scale_pulse_len = now - last;				\
    mb_scale_thrust = mb_scale_gain *				\
      ( mb_scale_pulse_len - mb_scale_neutral);			\
    last = now;							\
  }



#endif /* MB_SCALE_H */
