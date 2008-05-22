#include "booz_sensors_model.h"

#include BSM_PARAMS
#include "booz_sensors_model_utils.h"
#include "booz_flight_model.h"

bool_t booz_sensors_model_rangemeter_available() {
  if (bsm.rangemeter_available) {
    bsm.rangemeter_available = FALSE;
    return TRUE;
  }
  return FALSE;
}


void booz_sensors_model_rangemeter_init(double time) {

  bsm.rangemeter = 0.;

  bsm.rangemeter_next_update = time;
  bsm.rangemeter_available = FALSE;
  
}


void booz_sensors_model_rangemeter_run( double time ) {
  if (time < bsm.rangemeter_next_update)
    return;

  /* compute dist from ground */
  double dz = bfm.state->ve[BFMS_Z];
  if (dz > 0.) dz = 0.;
  double dx = dz * tan(bfm.state->ve[BFMS_THETA]);
  double dy = dz * tan(bfm.state->ve[BFMS_PHI]);
  double dist = sqrt( dx*dx + dy*dy + dz*dz);
  dist *= BSM_RANGEMETER_SENSITIVITY;
  /* add gaussian noise */

  if (dist > BSM_RANGEMETER_MAX_RANGE)
    dist = BSM_RANGEMETER_MAX_RANGE;
  dist = rint(dist);
  bsm.rangemeter = dist;

  bsm.rangemeter_next_update += BSM_RANGEMETER_DT;
  bsm.rangemeter_available = TRUE;

}
