#include "nps_radio_control.h"

#define RADIO_CONTROL_DT (1./40.)

struct NpsRadioControl nps_radio_control;

void nps_radio_control_init(void) {
  nps_radio_control.next_update = 0.;
}

#define MODE_SWITCH_MANUAL -1.0
#define MODE_SWITCH_AUTO1   0.0
#define MODE_SWITCH_AUTO2   1.0

void nps_radio_control_run_script(double time) {
  nps_radio_control.roll = 0.;
  nps_radio_control.pitch = 0.;
  /* starts motors */
  if (time < 1.) {
    nps_radio_control.yaw = 1.;
    nps_radio_control.throttle = 0.;
    nps_radio_control.mode = MODE_SWITCH_MANUAL;
  }
  else if (time < 3.5) {
    nps_radio_control.yaw = 0.;
    nps_radio_control.throttle = 0.;
    nps_radio_control.mode = MODE_SWITCH_MANUAL;
  }
  else {
    nps_radio_control.yaw = 0.;
    //    nps_radio_control.throttle = 0.99;
    //    nps_radio_control.mode = MODE_SWITCH_AUTO2;
    nps_radio_control.throttle = 0.265;
    nps_radio_control.mode = MODE_SWITCH_MANUAL;
  }
}

bool_t nps_radio_control_available(double time) {
  if (time >=  nps_radio_control.next_update) {
    nps_radio_control.next_update += RADIO_CONTROL_DT;
    nps_radio_control_run_script(time);
    return TRUE;
  }
  return FALSE;
}

