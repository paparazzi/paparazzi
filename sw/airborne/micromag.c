#include "micromag.h"

volatile uint8_t micromag_status;
volatile int16_t micromag_values[MM_NB_AXIS];

void micromag_init( void ) {
  
  micromag_hw_init();
  
  uint8_t i;
  for (i=0; i<MM_NB_AXIS; i++)
    micromag_values[i] = 0;
  micromag_status = MM_IDLE;
}

void micromag_reset() {
  micromag_status = MM_IDLE;
}
