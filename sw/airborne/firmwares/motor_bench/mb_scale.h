#ifndef MB_SCALE_H
#define MB_SCALE_H

#include "std.h"
#include "LPC21xx.h"

/* P0.29 CAP0.3 */
#define ICP_PINSEL     PINSEL1
#define ICP_PINSEL_VAL 0x02
#define ICP_PINSEL_BIT 26

extern volatile uint32_t mb_scale_pulse_len;
extern volatile float mb_scale_thrust;
extern volatile float mb_scale_torque;

extern volatile uint32_t mb_scale_neutral;
extern float    mb_scale_gain;
extern volatile uint8_t  mb_scale_calib;

#define MB_SCALE_NB_CALIB 50


void mb_scale_init(void);

#define MB_SCALE_IT TIR_CR3I
#define MB_SCALE_ICP_ISR() {                                            \
    static uint32_t last;                                               \
    uint32_t now = T0CR3;                                               \
    mb_scale_pulse_len = now - last;                                    \
    last = now;                                                         \
    if (mb_scale_calib > 0) {                                           \
      mb_scale_thrust += mb_scale_pulse_len;                            \
      if (mb_scale_calib >=  MB_SCALE_NB_CALIB) {                       \
        mb_scale_neutral = mb_scale_thrust /  MB_SCALE_NB_CALIB;        \
        mb_scale_calib = 0;                                             \
      }                                                                 \
      else                                                              \
        mb_scale_calib++;                                               \
    }                                                                   \
    else {                                                              \
      int32_t diff = (int32_t)mb_scale_pulse_len - (int32_t)mb_scale_neutral; \
      mb_scale_thrust = mb_scale_gain * diff;                           \
    }                                                                   \
  }

#define mb_scale_Calib(_val) {                  \
    mb_scale_calib = 1;                         \
    mb_scale_thrust = 0.;                       \
  }

#endif /* MB_SCALE_H */
