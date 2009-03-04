#ifndef NPS_JSBSIM_H
#define NPS_JSBSIM_H

#include <FGFDMExec.h>
#include "nps_fdm.h"

extern void nps_jsbsim_feed_inputs(JSBSim::FGFDMExec* fdmex, struct NpsFdmState* fdm_state);
extern void nps_jsbsim_fetch_state(JSBSim::FGFDMExec* fdmex, struct NpsFdmState* fdm_state);
extern int JSBInit(double sim_dt);

#endif /* NPS_JSBSIM_H */

