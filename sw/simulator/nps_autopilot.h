#ifndef NPS_AUTOPILOT_H
#define NPS_AUTOPILOT_H

#include "airframe.h"

#include "nps_radio_control.h"

struct NpsAutopilot {
  double commands[SERVOS_NB];
};

extern struct NpsAutopilot autopilot;

extern void nps_autopilot_init(enum NpsRadioControlType type, int num_script, char* js_dev);
extern void nps_autopilot_run_step(double time);

#endif /* NPS_AUTOPILOT_H */


