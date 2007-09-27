#include "mb_modes.h"

#include "adc.h"


uint8_t mb_modes_mode;
float mb_modes_throttle;


static struct adc_buf mb_modes_adc_buf; /* manual mode */

void mb_mode_init(void) {
  adc_buf_channel(1, &mb_modes_adc_buf, 16);
  mb_modes_mode = MB_MODES_MANUAL;
  mb_modes_throttle = 0.;
}


void mb_mode_event(void) {} 

void mb_mode_periodic(void) {

  uint16_t poti =  mb_modes_adc_buf.sum;

  switch (mb_modes_mode) {
  case MB_MODES_MANUAL :
    mb_modes_throttle = (float)poti/(16.*1024.);
    break;
  case MB_MODES_RAMP :
    mb_modes_throttle += 0.0001;
    if (mb_modes_throttle > 1.)
      mb_modes_throttle = 0.;
    break;
  default:
    mb_modes_throttle = 0.;
  }
}

