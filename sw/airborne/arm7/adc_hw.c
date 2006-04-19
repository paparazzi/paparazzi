#include "adc.h"

#include "LPC21xx.h"
#include "armVIC.h"

static struct adc_buf* buffers[NB_ADC];

volatile uint16_t adc0_val[NB_ADC] = {0, 0, 0, 0, 0, 0, 0, 0};
volatile uint16_t adc1_val[NB_ADC] = {0, 0, 0, 0, 0, 0, 0, 0};

void adcISR0 ( void ) __attribute__((naked));
void adcISR1 ( void ) __attribute__((naked));

void adc_buf_channel(uint8_t adc_channel, struct adc_buf* s, uint8_t av_nb_sample) {
  buffers[adc_channel] = s;
  s->av_nb_sample = av_nb_sample;
}

void adc_init( void ) {
  //  return;

  /* AD0.6 as ADC */
  PINSEL1 |= 3 << 8;
  /* AD0.6 - PCLK/4 - BURST ON */
  AD0CR = 1<<6 | 0x03 << 8 | 1 << 16 | 0x01 << 21 ;
  /* AD0 selected as IRQ */
  VICIntSelect &= ~VIC_BIT(VIC_AD0);
  /* AD0 interrupt enabled */
  VICIntEnable = VIC_BIT(VIC_AD0);  
  /* AD0 interrupt as VIC2 */
  VICVectCntl2 = VIC_ENABLE | VIC_AD0;
  VICVectAddr2 = (uint32_t)adcISR0;


}

void adcISR0 ( void ) {
  ISR_ENTRY();
  uint32_t tmp = AD0DR;
  uint8_t channel = (uint8_t)(tmp >> 24) & 0x07;
  adc0_val[channel] = (uint16_t)(tmp >> 6) & 0x03FF;
  VICVectAddr = 0x00000000;                 // clear this interrupt from the VIC
  ISR_EXIT();                               // recover registers and return
}

void adcISR1 ( void ) {
  ISR_ENTRY();
  uint32_t tmp = AD1DR;
  uint8_t channel = (uint8_t)(tmp >> 24) & 0x07;
  adc1_val[channel] = (uint16_t)(tmp >> 6) & 0x03FF;
  VICVectAddr = 0x00000000;                 // clear this interrupt from the VIC
  ISR_EXIT();                               // recover registers and return
}
