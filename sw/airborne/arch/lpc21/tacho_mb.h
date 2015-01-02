#ifndef TACHO_MB_H
#define TACHO_MB_H

#include "std.h"

extern void tacho_mb_init(void);

extern uint32_t t_duration;

#define TACHO_MB_ISR() {            \
    static uint32_t tmb_last;           \
    uint32_t t_now = T0CR0;           \
    t_duration = t_now - tmb_last;          \
    tmb_last = t_now;             \
    LED_TOGGLE(1);              \
  }

#endif /* TACHO_MB_H */
