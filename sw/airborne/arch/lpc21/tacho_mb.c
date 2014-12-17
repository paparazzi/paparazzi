#include "tacho_mb.h"

#include "LPC21xx.h"

uint32_t t_duration;

/* INPUT CAPTURE CAP0.0 on P0.22*/
#define TACHO_MB_PINSEL     PINSEL1
#define TACHO_MB_PINSEL_VAL 0x02
#define TACHO_MB_PINSEL_BIT 12

void tacho_mb_init(void)
{
  /* select pin for capture */
  TACHO_MB_PINSEL |= TACHO_MB_PINSEL_VAL << TACHO_MB_PINSEL_BIT;
  /* enable capture 0.2 on falling edge + trigger interrupt */
  T0CCR |= TCCR_CR0_F | TCCR_CR0_I;
}
