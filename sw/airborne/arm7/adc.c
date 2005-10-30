#include "adc.h"
#include "LPC21xx.h"
#include "armVIC.h"

volatile uint16_t adc_val = 000;

void adcInit ( void ) {
  PINSEL1 |= 0x01 << 22 ; /* P0.27 is AD0.0 */
  AD0CR = 0x01 | 0x03 << 8 | 0 << 16 | 0x01 << 21 ; /* AD0.0 - PCLK/4 - BURST OFF */

  //#if 0
  VICIntSelect &= ~VIC_BIT(VIC_AD0);  // AD0 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_AD0);    // AD0 interrupt enabled
  VICVectCntl2 = VIC_ENABLE | VIC_AD0;
  VICVectAddr2 = (uint32_t)adcISR;    // address of the ISR
  //#endif
  AD0CR |= 0x01 << 24;  
}

void adcISR ( void ) {
  // perform proper ISR entry so thumb-interwork works properly
  ISR_ENTRY();
  adc_val = (uint16_t)(AD0DR >> 6) & 0x03FF;
  //  adc_val = 500;
  AD0CR |= 0x01 << 24;                      /* Start A/D Conversion           */
  VICVectAddr = 0x00000000;             // clear this interrupt from the VIC
  ISR_EXIT();                           // recover registers and return
}

uint16_t adcPoll (void) {
  uint32_t tmp;
  AD0CR |= 0x01 << 24;                      /* Start A/D Conversion           */
  do {
    tmp = AD0DR;                            /* Read A/D Data Register         */
  } while ((tmp & (0x1 << 31)) == 0);       /* Wait for end of A/D Conversion */
  return (tmp >> 6) & 0x03FF;               /* Extract AIN0 Value             */
}

