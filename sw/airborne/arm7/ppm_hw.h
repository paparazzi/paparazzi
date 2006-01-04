#ifndef PPM_HW_H
#define PPM_HW_H

#include "LPC21xx.h"
#include CONFIG

void TIMER0_ISR ( void ) __attribute__((naked));

static inline void ppm_init ( void ) {
  /* select TIMER0 as IRQ    */
  VICIntSelect &= ~VIC_BIT(VIC_TIMER0);
  /* enable TIMER0 interrupt */
  VICIntEnable = VIC_BIT(VIC_TIMER0); 
  /* on slot vic slot 4      */
  VICVectCntl4 = VIC_ENABLE | VIC_TIMER0;
  /* address of the ISR      */
  VICVectAddr4 = (uint32_t)TIMER0_ISR; 
  /* select pin for capture */
  PPM_PINSEL |= PPM_PINSEL_VAL << PPM_PINSEL_BIT;
  /* enable capture 0.2 on rising edge + trigger interrupt */
  T0CCR = TCCR_CR2_R | TCCR_CR2_I;

  ppm_valid = FALSE;
}


#endif /* PPM_HW_H */
