#ifndef NPS_AUTOPILOT_H
#define NPS_AUTOPILOT_H

#include "generated/airframe.h"

#include "nps_radio_control.h"

struct NpsAutopilot {
  double commands[COMMANDS_NB];
};

extern struct NpsAutopilot autopilot;

extern bool_t nps_bypass_ahrs;
extern bool_t nps_bypass_ins;
extern void sim_overwrite_ahrs(void);
extern void sim_overwrite_ins(void);

extern void nps_autopilot_init(enum NpsRadioControlType type, int num_script, char* js_dev);
extern void nps_autopilot_run_step(double time);
extern void nps_autopilot_run_systime_step(void);


#endif /* NPS_AUTOPILOT_H */

