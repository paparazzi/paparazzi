#ifndef NPS_AUTOPILOT_ROTORCRAFT_H
#define NPS_AUTOPILOT_ROTORCRAFT_H

#include "nps_autopilot.h"


extern bool_t nps_bypass_ahrs;
extern bool_t nps_bypass_ins;
extern void sim_overwrite_ahrs(void);
extern void sim_overwrite_ins(void);

#endif /* NPS_AUTOPILOT_ROTORCRAFT_H */
