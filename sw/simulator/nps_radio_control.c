#include "nps_radio_control.h"

#define RADIO_CONTROL_DT (1./40.)

struct NpsRadioControl nps_radio_control;

void nps_radio_control_init(void) {
  nps_radio_control.next_update = 0.;
}

static void radio_control_script_step_roll(double time);
static void radio_control_script_ff(double time);

#define MODE_SWITCH_MANUAL -1.0
#define MODE_SWITCH_AUTO1   0.0
#define MODE_SWITCH_AUTO2   1.0

#define RADIO_CONTROL_TAKEOFF_TIME 8

void radio_control_script_takeoff(double time) {
  
  nps_radio_control.roll = 0.;
  nps_radio_control.pitch = 0.;
  nps_radio_control.yaw = 0.;
  nps_radio_control.throttle = 0.;
  nps_radio_control.mode = MODE_SWITCH_MANUAL;
  /* starts motors */
  if (time < 1.)
    nps_radio_control.yaw = 1.;
  else
    nps_radio_control.yaw = 0.;

}

void radio_control_script_step_roll(double time) {
  nps_radio_control.throttle = 0.99;
  nps_radio_control.mode = MODE_SWITCH_AUTO2;
  
  if (((int32_t)rint((time*0.5)))%2) {
    nps_radio_control.roll = 0.2;
    nps_radio_control.yaw = 0.5;
  }
  else
    nps_radio_control.roll = RadOfDeg(10);
}

void radio_control_script_ff(double time __attribute__ ((unused))) {
  nps_radio_control.throttle = 0.99;
  nps_radio_control.mode = MODE_SWITCH_AUTO2;
  if (time < RADIO_CONTROL_TAKEOFF_TIME+3)
    nps_radio_control.pitch = -1.;
  else if (time < RADIO_CONTROL_TAKEOFF_TIME+6) {
    //    nps_radio_control.roll = 0.5;
    nps_radio_control.pitch = -1.;
  }
  else {
    nps_radio_control.pitch = 0.;
    nps_radio_control.roll = 0.;
  }
}



void nps_radio_control_run_script(double time) {
  if (time < RADIO_CONTROL_TAKEOFF_TIME)
    radio_control_script_takeoff(time);
  else {
    //    radio_control_script_step_roll(time);
    radio_control_script_ff(time);
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

