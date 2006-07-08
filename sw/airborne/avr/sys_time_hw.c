#include CONFIG
#include "std.h"

#include <inttypes.h>
#include <avr/interrupt.h>

#if CLOCK == 8
volatile uint8_t tmr2_ov_cnt;
volatile bool_t tmr2_overflow;

SIGNAL(SIG_OVERFLOW2) {
  tmr2_ov_cnt++;
  tmr2_overflow = TRUE;
  return;
}

#endif
