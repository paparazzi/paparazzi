#include "nps_radio_control.h"

#define RADIO_CONTROL_DT (1./40.)

struct NpsRadioControl nps_radio_control;

void nps_radio_control_init(void) {
  nps_radio_control.next_update = 0.;
}


bool_t nps_radio_control_available(double time) {
  if (time >=  nps_radio_control.next_update) {
    nps_radio_control.next_update += RADIO_CONTROL_DT;

    



    return TRUE;
  }
  return FALSE;
}

