#ifndef NPS_AUTOPILOT_H
#define NPS_AUTOPILOT_H

#include "airframe.h"

struct NpsAutopilot {
  double commands[SERVOS_NB];
};

extern struct NpsAutopilot autopilot;

extern void nps_autopilot_run_step(void);

#endif /* NPS_AUTOPILOT_H */


