#ifndef MB_TACHO_H
#define MB_TACHO_H

#include "std.h"

extern void mb_tacho_init(void);
extern uint32_t mb_tacho_get_duration(void);
extern float mb_tacho_get_averaged(void);

extern volatile uint32_t mb_tacho_duration;
extern volatile uint8_t got_one_pulse;
extern volatile float mb_tacho_averaged;
extern volatile uint16_t  mb_tacho_nb_pulse;

#define MB_TACHO_IT TIR_CR0I
#define MB_TACHO_ISR() {            \
    static uint32_t tmb_last;           \
    uint32_t t_now = T0CR0;           \
    uint32_t diff = t_now - tmb_last;         \
    mb_tacho_duration = diff;           \
    mb_tacho_averaged += diff;            \
    mb_tacho_nb_pulse++;            \
    tmb_last = t_now;             \
    got_one_pulse = TRUE;                                               \
  }

#endif /* MB_TACHO_H */
