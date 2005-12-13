#include "adc.h"
#include "LPC21xx.h"
#include "armVIC.h"

volatile uint16_t adc0_val[ADC_NB_CHAN] = {0, 0, 0, 0, 0, 0};
volatile uint16_t adc1_val[ADC_NB_CHAN] = {0, 0, 0, 0, 0, 0};

void adc_init ( void ) {
  /* AD0.0 to AD0.3 as ADC */
  //  PINSEL1 |= 1 << 22 | 1 << 24 | 1 << 26 | 1 << 28;
  PINSEL1 |= 1 << 12 | 1 << 22 | 1 << 24 | 1 << 26 | 1 << 28;
  /* AD0.0 to AD0.3 - PCLK/4 - BURST ON */
  AD0CR = 0x0F | 0x03 << 8 | 1 << 16 | 0x01 << 21 ;

  VICIntSelect &= ~VIC_BIT(VIC_AD0);   // AD0 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_AD0);     // AD0 interrupt enabled
  VICVectCntl2 = VIC_ENABLE | VIC_AD0;
  VICVectAddr2 = (uint32_t)adcISR0;    // address of the ISR

  /* AD1.0 to AD1.3 as ADC */
  //PINSEL0 |= 3 << 12 ;//| 3 << 16 | 3 << 20 | 3 << 24;
  //  PINSEL1 |= 1 << 12 ;
  /* AD1.0 to AD1.3 - PCLK/4 - BURST ON */
  //  AD1CR = 0x80 | 0x03 << 8 | 1 << 16 | 0x01 << 21 ; 

  //  VICIntSelect &= ~VIC_BIT(VIC_AD1);   // AD1 selected as IRQ
  //  VICIntEnable = VIC_BIT(VIC_AD1);     // AD1 interrupt enabled
  //  VICVectCntl3 = VIC_ENABLE | VIC_AD1;
  //  VICVectAddr3 = (uint32_t)adcISR1;    // address of the ISR
}

void adcISR0 ( void ) {
  // perform proper ISR entry so thumb-interwork works properly
  ISR_ENTRY();
  uint32_t tmp = AD0DR;
  uint8_t channel = (uint8_t)(tmp >> 24) & 0x07;
  adc0_val[channel] = (uint16_t)(tmp >> 6) & 0x03FF;
  VICVectAddr = 0x00000000;                 // clear this interrupt from the VIC
  ISR_EXIT();                               // recover registers and return
}

void adcISR1 ( void ) {
  // perform proper ISR entry so thumb-interwork works properly
  ISR_ENTRY();
  uint32_t tmp = AD1DR;
  uint8_t channel = (uint8_t)(tmp >> 24) & 0x07;
  adc1_val[channel] = (uint16_t)(tmp >> 6) & 0x03FF;
  VICVectAddr = 0x00000000;                 // clear this interrupt from the VIC
  ISR_EXIT();                               // recover registers and return
}
