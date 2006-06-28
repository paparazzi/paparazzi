#include "servos_direct_hw.h"

/* 40 Hz */
#define SERVOS_PERIOD SERVOS_TICS_OF_USECS(25000);


void actuators_init ( void ) {

  /* configure pins for PWM */
  PINSEL0 |= 2 << 0 | 2 << 2 | 2 << 14 | 2 << 16 | 2 << 18;
  PINSEL1 |= 2 << 10;

  /* set servo refresh rate */
  PWMMR0 = SERVOS_PERIOD;

  /* enable all 6 PWM outputs in single edge mode*/
  PWMPCR = PWMENA1 | PWMENA2 | PWMENA3 | PWMENA4 | PWMENA5 | PWMENA6;

  /* commit PWMMRx changes */
  PWMLER = PWMLER_LATCH0;

  /* enable PWM timer in PWM mode */
  PWMTCR = PWMTCR_COUNTER_ENABLE | PWMTCR_PWM_ENABLE;


}
