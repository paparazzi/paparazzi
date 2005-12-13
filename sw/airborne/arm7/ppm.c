#include "ppm.h"
#include "LPC21xx.h"
#include "armVIC.h"
#include "config.h"

uint16_t ppm_pulses[PPM_NB_CHANNEL];
volatile uint8_t ppm_valid = FALSE;

void ppm_init ( void ) {
  /* select TIMER0 as IRQ    */
  VICIntSelect &= ~VIC_BIT(VIC_TIMER0);
  /* enable TIMER0 interrupt */
  VICIntEnable = VIC_BIT(VIC_TIMER0); 
  /* on slot vic slot 4      */
  VICVectCntl4 = VIC_ENABLE | VIC_TIMER0;
  /* address of the ISR      */
  VICVectAddr4 = (uint32_t)TIMER0_ISR; 
  /* select P0.6 as capture */
  PINSEL0 |= 0x02 << 12;
  /* enable capture 0.2 on rising edge + trigger interrupt */
  T0CCR = 1 << 6 | 1 << 8;
}

void TIMER0_ISR ( void ) {
  ISR_ENTRY();

  static uint8_t state = PPM_NB_CHANNEL;
  static uint32_t last;
  
  uint32_t now = T0CR2;
  uint32_t length = now - last;
  last = now;

  if (state == PPM_NB_CHANNEL) {
    if (length > PPM_SYNC_MIN_LEN && length < PPM_SYNC_MAX_LEN) {
      state = 0;
    }
  }
  else {
    if (length > PPM_DATA_MIN_LEN && length < PPM_DATA_MAX_LEN) {
      ppm_pulses[state] = length;
      state++;
      if (state == PPM_NB_CHANNEL) {
	ppm_valid = TRUE;
      }
    }
    else
      state = PPM_NB_CHANNEL;
  }

  /* clear interrupt */
  T0IR = 1<<6;
  VICVectAddr = 0x00000000;
  ISR_EXIT();
}
