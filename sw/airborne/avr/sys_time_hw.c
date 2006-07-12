#include <inttypes.h>
#if (__GNUC__ == 3)
#include <avr/signal.h>
#endif
#include <avr/interrupt.h>

#include CONFIG
#include "std.h"

#if CLOCK == 8
volatile uint8_t tmr2_ov_cnt;
volatile bool_t tmr2_overflow;

SIGNAL(SIG_OVERFLOW2) {
  tmr2_ov_cnt++;
  tmr2_overflow = TRUE;
  return;
}

#endif
