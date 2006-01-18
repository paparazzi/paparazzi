#ifndef MODEM_HW_H
#define MODEM_HW_H

#include "LPC21xx.h"
#include CONFIG

void TIMER1_ISR ( void ) __attribute__((naked));


#define T1_PCLK_DIV 3
#define SAMPLES_PER_PERIOD 4
#define SAMPLE_PERIOD (PCLK/4762/SAMPLES_PER_PERIOD/T1_PCLK_DIV)

static inline void modem_init ( void ) {
    /* turn on DAC pins */
  PINSEL1 &= 1 << 19;
  PINSEL1 |= ~(1 << 18);
  /* reset & disable timer 1   */
  T1TCR = TCR_RESET;
  /* set the prescale divider  */
  T1PR = T1_PCLK_DIV - 1;
 /* select TIMER1 as IRQ       */
  VICIntSelect &= ~VIC_BIT(VIC_TIMER1);
  /* enable TIMER1 interrupt   */
  VICIntEnable = VIC_BIT(VIC_TIMER1);
  /* on slot vic slot 1        */
  VICVectCntl1 = VIC_ENABLE | VIC_TIMER1;
  /* address of the ISR        */
  VICVectAddr1 = (uint32_t)TIMER1_ISR;
  /* trigger initial match in a long time from now */
  T1MR0 = SAMPLE_PERIOD;
  /* enable interrupt on match register 0 */
  T1MCR |= TMCR_MR0_I | TMCR_MR0_R;
  /* enable timer 1 */
  T1TCR = TCR_ENABLE; 
}


#define MODEM_CHECK_RUNNING() {}


#endif /* MODEM_HW_H */
