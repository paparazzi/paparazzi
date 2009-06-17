
#include <FGFDMExec.h>
#include <FGJSBBase.h>
#include <FGState.h>

#include "nps_fdm.h"

#include "airframe.h"

using namespace JSBSim;

static void feed_jsbsim(double* commands);
static void fetch_state(void);

FGFDMExec* FDMExec;

void nps_fdm_init(double dt) {

  // FDMExec = new FGFDMExec();

}

void nps_fdm_run_step(double* commands) {

  feed_jsbsim(commands);

  /* run JSBSim */

  fetch_state();

}

static void feed_jsbsim(double* commands) {

#if 0
  printf("Actuators Names:\n");
  for (int i=0; i<SERVOS_NB; i++) {
    printf("%s\n", ACTUATOR_COMMAND_NAMES[i]);
  }
#endif

}

static void fetch_state(void) {

  /* JSBSim to fdm */

}
