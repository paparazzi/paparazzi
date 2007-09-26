#include "mb_modes.h"


uint8_t mb_modes_mode;
float mb_modes_throttle;

void mb_mode_init(void) {
  mb_modes_mode = MB_MODES_IDLE;
  mb_modes_throttle = 0.;
}


void mb_mode_event(void) {} 


void mb_mode_periodic(void) {
  mb_modes_throttle += 0.01;
  if (mb_modes_throttle > 1.)
    mb_modes_throttle = 0.;
}

