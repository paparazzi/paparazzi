#include "booz2_micromag.h"

volatile uint8_t booz2_micromag_status;
volatile int16_t booz2_micromag_values[MM_NB_AXIS];

uint8_t do_booz2_micromag_read;

void booz2_micromag_init( void ) {
  
  booz2_micromag_hw_init();

  do_booz2_micromag_read = false;
  
  uint8_t i;
  for (i=0; i<MM_NB_AXIS; i++)
    booz2_micromag_values[i] = 0;
  booz2_micromag_status = MM_IDLE;
}

void booz2_micromag_reset() {
  booz2_micromag_status = MM_IDLE;
}

#include "led.h"
void booz2_micromag_read() {
  if (booz2_micromag_status == MM_IDLE) {
    MmSendReq();
  }
  else if (booz2_micromag_status ==  MM_GOT_EOC) {
    MmReadRes();
  }
}

