#include "booz_sensors_model.h"

#include BSM_PARAMS
#include "booz_sensors_model_utils.h"
#include "booz_flight_model.h"

bool_t booz_sensors_model_baro_available() {
  if (bsm.baro_available) {
    bsm.baro_available = FALSE;
    return TRUE;
  }
  return FALSE;
}


void booz_sensors_model_baro_init( double time ) {
  bsm.baro = 0.;

  bsm.baro_next_update = time;
  bsm.baro_available = FALSE;

}

void booz_sensors_model_baro_run( double time ) {
  if (time < bsm.baro_next_update)
    return;

  if (time < 12.5)
    bsm.baro = 840;
  else {
    double z = bfm.pos_ltp->ve[AXIS_Z] + get_gaussian_noise()*BSM_BARO_NOISE_STD_DEV;
    //  double p = ( z / 0.084 ) + BSM_BARO_QNH;
    //  double baro_reading = p * BSM_BARO_SENSITIVITY;
    double baro_reading = BSM_BARO_QNH + z * BSM_BARO_SENSITIVITY;

    /* FIXME : add noise and random walk */
    baro_reading = rint(baro_reading);
    bsm.baro = baro_reading;
  }
  bsm.baro_next_update += BSM_BARO_DT;
  bsm.baro_available = TRUE;
}
