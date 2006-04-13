#ifndef SERVOS_4015_HW_H
#define SERVOS_4015_HW_H

#include "std.h"
#include "LPC21xx.h"
#include CONFIG



void PWM_ISR ( void ) __attribute__((naked));

static inline void command_init ( void ) {
  /* PWM selected as IRQ */
  VICIntSelect &= ~VIC_BIT(VIC_PWM);   
  /* PWM interrupt enabled */
  VICIntEnable = VIC_BIT(VIC_PWM);  
  VICVectCntl3 = VIC_ENABLE | VIC_PWM;
  /* address of the ISR */
  VICVectAddr3 = (uint32_t)PWM_ISR;
  /* PW5 pin (P0.21) used for PWM  */
  IO0DIR |= _BV(SERV0_CLOCK_PIN);
  IO1DIR |= _BV(SERV0_DATA_PIN) | _BV(SERV0_RESET_PIN);
  SERV0_CLOCK_PINSEL |= SERV0_CLOCK_PINSEL_VAL << SERV0_CLOCK_PINSEL_BIT;

  /* set match5 to go of a long time from now */
  PWMMR0 = 0XFFFFFF;  
  //PWMMR0 = CLOCK_OF_US(1500);  
  PWMMR5 = 0XFFF;  
  /* commit above change        */
  PWMLER = PWMLER_LATCH0 | PWMLER_LATCH5;
  /* interrupt on PWMMR5 match  */
  PWMMCR = PWMMCR_MR0R | PWMMCR_MR5I;
  /* enable PWM5 ouptput        */
  PWMPCR = PWMPCR_ENA5;
  /* enable PWM timer counter and PWM mode  */
  PWMTCR = PWMTCR_COUNTER_ENABLE | PWMTCR_PWM_ENABLE; 
  /* Load failsafe values              */
  command_set(failsafe_values); 
}

#endif /* SERVOS_4015_HW_H */
