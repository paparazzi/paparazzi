
#include "inttypes.h"
#include "lpc2138.h"
#include "paparazzi_board.h"

/*
 PWM1 P0.0
 PWM2 P0.7
 PWM3 P0.1
 PWM4 P0.8
 PWM5 P0.21
 PWM6 P0.9
*/

#define PWM_PINSEL0_MASK 0x02 << 0 | 0x02 << 14 | 0x02 << 2 | 0x02 << 16 | 0x02 << 18
#define PWM_PINSEL1_MASK 0x02 << 10
#define PWM_NB_CHANNEL 6
/* pwm frequency in hertz */
#define PWM_FREQ  60

void pwm_init ( void ) {
  /* all PWM pins used for PWM                  */
  PINSEL0 |= PWM_PINSEL0_MASK;
  PINSEL1 |= PWM_PINSEL1_MASK;
  /* PWM timer reset => PWM freq                */
  PWMMR0 = PCLK / PWM_FREQ; 
  PWMMR1 = 25000;  /* duty 1/2                  */
  PWMLER = 0x3;    /* commit above changes      */
  PWMMCR = 0x2;    /* pwm timer reset on PWMMR0 */
  /* pwm1 to pwm6 output enabled                */
  PWMPCR = 1 << 9 | 1 << 10 | 1 << 11 | 1 << 12 | 1 << 13 | 1 << 14 ; 
  PWMPR =  0x0;    /* no prescaler              */
  PWMTCR = 0x9;    /* enable timer and PWM      */
}

void pwm_set ( uint8_t channel, uint32_t value) {
  volatile uint32_t* pwm_reg = &PWMMR0 + channel * sizeof(PWMMR0);
  *pwm_reg = value;
  PWMLER = 0x3;    /* commit above change       */
}
