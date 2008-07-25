#include "lpc_vor_convertions.h"

#include "LPC21xx.h"
#include "armVIC.h"

#include "sys_time.h"

#include "led.h"

volatile uint16_t vor_adc_sample;
volatile bool_t vor_adc_sample_available;
volatile uint32_t vor_adc_overrun;


void adcISR0 ( void ) __attribute__((naked));

// AD0.6 on P0.4
void vor_adc_init( void ) {
  /* select P0.4 as ADC */
  PINSEL0 |=  3 << 8;
  /* sample AD0.6 - PCLK/4 ( 3.75MHz) - ON */
  AD0CR = 1 << 6 | 0x03 << 8 | 1 << 21;// | 1<<16;
  /* AD0 selected as IRQ */
  VICIntSelect &= ~VIC_BIT(VIC_AD0);
  /* AD0 interrupt enabled */
  VICIntEnable = VIC_BIT(VIC_AD0);
  /* AD0 interrupt as VIC2 */
  VICVectCntl2 = VIC_ENABLE | VIC_AD0;
  VICVectAddr2 = (uint32_t)adcISR0;
  /* start convertion on T0M1 match */
  ADGSR = 4 << 24;

  /* clear match */					
  T0EMR &= ~TEMR_EM1;					
  /* set high on match 1 */
  T0EMR |= TEMR_EMC11;
  /* first match in a while */
  T0MR1 = 1024;
  vor_adc_overrun = FALSE;
}


void adcISR0 ( void ) {
  ISR_ENTRY();
  uint32_t tmp = AD0GDR;
  //  uint32_t tmp = AD0DR3;
  //  uint8_t  channel = (uint8_t)(tmp >> 24) & 0x07;
  vor_adc_sample = (uint16_t)(tmp >> 6) & 0x03FF;
  if (vor_adc_sample_available) {
    vor_adc_overrun++;
    LED_ON(1);
  }
  vor_adc_sample_available = TRUE;

  /* trigger next convertion */
  T0MR1 += PERIODIC_TASK_PERIOD;
  /* lower clock         */
  T0EMR &= ~TEMR_EM1;	

  VICVectAddr = 0x00000000;                 // clear this interrupt from the VIC
  ISR_EXIT();                               // recover registers and return
}
