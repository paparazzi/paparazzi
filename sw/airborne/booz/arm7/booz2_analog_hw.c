#include "booz2_analog.h"

#include "armVIC.h"
#include "sys_time.h"

#include "booz2_analog_baro.h"
#include "booz2_battery.h"

void ADC0_ISR ( void ) __attribute__((naked));
void ADC1_ISR ( void ) __attribute__((naked));

void booz2_analog_init_hw( void ) {

  /* start ADC0 */
  /* select P0.29 as AD0.2 for bat meas*/
  PINSEL1 |=  0x01 << 26;
  /* sample AD0.2 - PCLK/4 ( 3.75MHz) - ON */
  AD0CR = 1 << 2 | 0x03 << 8 | 1 << 21;
  /* AD0 selected as IRQ */
  VICIntSelect &= ~VIC_BIT(VIC_AD0);
  /* AD0 interrupt enabled */
  VICIntEnable = VIC_BIT(VIC_AD0);
  /* AD0 interrupt as VIC2 */
  _VIC_CNTL(ADC0_VIC_SLOT) = VIC_ENABLE | VIC_AD0;
  _VIC_ADDR(ADC0_VIC_SLOT) = (uint32_t)ADC0_ISR;
  /* start convertion on T0M1 match */
  AD0CR |= 4 << 24;


 /* clear match 1 */                                     
  T0EMR &= ~TEMR_EM1;                                   
  /* set high on match 1 */
  T0EMR |= TEMR_EMC1_2;
  /* first match in a while */
  T0MR1 = 1024;


  /* start ADC1 */
  /* select P0.10 as AD1.2 for baro*/
  ANALOG_BARO_PINSEL |=  ANALOG_BARO_PINSEL_VAL << ANALOG_BARO_PINSEL_BIT;
  /* sample AD1.2 - PCLK/4 ( 3.75MHz) - ON */
  AD1CR = 1 << 2 | 0x03 << 8 | 1 << 21;
  /* AD0 selected as IRQ */
  VICIntSelect &= ~VIC_BIT(VIC_AD1);
  /* AD0 interrupt enabled */
  VICIntEnable = VIC_BIT(VIC_AD1);
  /* AD0 interrupt as VIC2 */
  _VIC_CNTL(ADC1_VIC_SLOT) = VIC_ENABLE | VIC_AD1;
  _VIC_ADDR(ADC1_VIC_SLOT) = (uint32_t)ADC1_ISR;
  /* start convertion on T0M3 match */
  AD1CR |= 5 << 24;


  /* clear match 2 */                                     
  T0EMR &= ~TEMR_EM3;                                   
  /* set high on match 2 */
  T0EMR |= TEMR_EMC3_2;
  /* first match in a while */
  T0MR3 = 512;

  /* turn on DAC pins */
  PINSEL1 |= 2 << 18;
}


void ADC0_ISR ( void ) {
  ISR_ENTRY();
  uint32_t tmp = AD0GDR;
  uint16_t tmp2 = (uint16_t)(tmp >> 6) & 0x03FF;
  Booz2BatteryISRHandler(tmp2);
  /* trigger next convertion */
  T0MR1 += BOOZ2_ANALOG_BATTERY_PERIOD;
  /* lower clock         */
  T0EMR &= ~TEMR_EM1;   
  VICVectAddr = 0x00000000;                 // clear this interrupt from the VIC
  ISR_EXIT();                               // recover registers and return
}

void ADC1_ISR ( void ) {
  ISR_ENTRY();
  uint32_t tmp = AD1GDR;
  uint16_t tmp2 = (uint16_t)(tmp >> 6) & 0x03FF;
  Booz2BaroISRHandler(tmp2);
  /* trigger next convertion */
  T0MR3 += BOOZ2_ANALOG_BARO_PERIOD;
  /* lower clock         */
  T0EMR &= ~TEMR_EM3;   
  VICVectAddr = 0x00000000;                 // clear this interrupt from the VIC
  ISR_EXIT();                               // recover registers and return
}

