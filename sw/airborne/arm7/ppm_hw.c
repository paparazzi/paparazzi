#include "ppm.h"
#include "armVIC.h"
#include "std.h"
#include "sys_time.h"
#include CONFIG

#define PPM_NB_CHANNEL PPM_NB_PULSES

uint16_t ppm_pulses[PPM_NB_CHANNEL];
volatile bool_t ppm_valid;


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
  T0IR = TIR_CR2I;
  VICVectAddr = 0x00000000;
  ISR_EXIT();
}
