#include "std.h"
#include "actuators.h"
#include "servos_direct_hw.h"

/* 40 Hz */
#ifndef SERVOS_PERIOD
#define SERVOS_PERIOD SERVOS_TICS_OF_USEC(25000)
#endif

const uint8_t pwm_latch_value = 0
#if defined PWM_SERVO_0
    | PWM_SERVO_0_LATCH 
#endif
#if defined PWM_SERVO_1
    | PWM_SERVO_1_LATCH 
#endif
#if defined PWM_SERVO_2
    | PWM_SERVO_2_LATCH 
#endif
#if defined PWM_SERVO_3
    | PWM_SERVO_3_LATCH 
#endif
#if defined PWM_SERVO_4
    | PWM_SERVO_4_LATCH 
#endif
#if defined PWM_SERVO_5
    | PWM_SERVO_5_LATCH 
#endif
  ;

void actuators_init ( void ) {

  /* configure pins for PWM */
#if defined PWM_SERVO_0
  PWM_SERVO_0_PINSEL = (PWM_SERVO_0_PINSEL & PWM_SERVO_0_PINSEL_MASK) | PWM_SERVO_0_PINSEL_VAL << PWM_SERVO_0_PINSEL_BIT;
#endif
#if defined PWM_SERVO_1
  PWM_SERVO_1_PINSEL = (PWM_SERVO_1_PINSEL & PWM_SERVO_1_PINSEL_MASK) | PWM_SERVO_1_PINSEL_VAL << PWM_SERVO_1_PINSEL_BIT;
#endif
#if defined PWM_SERVO_2
  PWM_SERVO_2_PINSEL = (PWM_SERVO_2_PINSEL & PWM_SERVO_2_PINSEL_MASK) | PWM_SERVO_2_PINSEL_VAL << PWM_SERVO_2_PINSEL_BIT;
#endif
#if defined PWM_SERVO_3
  PWM_SERVO_3_PINSEL = (PWM_SERVO_3_PINSEL & PWM_SERVO_3_PINSEL_MASK) | PWM_SERVO_3_PINSEL_VAL << PWM_SERVO_3_PINSEL_BIT;
#endif
#if defined PWM_SERVO_4
  PWM_SERVO_4_PINSEL = (PWM_SERVO_4_PINSEL & PWM_SERVO_4_PINSEL_MASK) | PWM_SERVO_4_PINSEL_VAL << PWM_SERVO_4_PINSEL_BIT;
#endif
#if defined PWM_SERVO_5
  PWM_SERVO_5_PINSEL = (PWM_SERVO_5_PINSEL & PWM_SERVO_5_PINSEL_MASK) | PWM_SERVO_5_PINSEL_VAL << PWM_SERVO_5_PINSEL_BIT;
#endif

  /* set servo refresh rate */
  PWMMR0 = SERVOS_PERIOD;

  /* FIXME: For now, this prescaler needs to match the TIMER0 prescaler, as the
  higher level code treats them the same */
  PWMPR = 1;

  /* enable all 6 PWM outputs in single edge mode*/
  PWMPCR = 0
#if defined PWM_SERVO_0
    | PWM_SERVO_0_ENA 
#endif
#if defined PWM_SERVO_1
    | PWM_SERVO_1_ENA 
#endif
#if defined PWM_SERVO_2
    | PWM_SERVO_2_ENA 
#endif
#if defined PWM_SERVO_3
    | PWM_SERVO_3_ENA 
#endif
#if defined PWM_SERVO_4
    | PWM_SERVO_4_ENA 
#endif
#if defined PWM_SERVO_5
    | PWM_SERVO_5_ENA 
#endif
    ;

  /* commit PWMMRx changes */
  PWMLER = PWMLER_LATCH0;

  /* enable PWM timer in PWM mode */
  PWMTCR = PWMTCR_COUNTER_ENABLE | PWMTCR_PWM_ENABLE;


}
