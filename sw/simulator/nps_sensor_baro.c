#include "nps_sensor_baro.h"

#include "airframe.h"

#include "std.h"
#include "nps_fdm.h"
#include "nps_random.h"
#include NPS_SENSORS_PARAMS



void nps_sensor_baro_init(struct NpsSensorBaro* baro, double time) {
  baro->value = 0.;
  baro->next_update = time;
  baro->data_available = FALSE;
}


void nps_sensor_baro_run_step(struct NpsSensorBaro* baro, double time) {
  
  if (time < baro->next_update)
    return;

  if (time < 10.)
    baro->value = rint(time*90);
  else {
    double z = fdm.ltpprz_pos.z + get_gaussian_noise()*NPS_BARO_NOISE_STD_DEV;
    double baro_reading = NPS_BARO_QNH + z * NPS_BARO_SENSITIVITY;
    baro_reading = rint(baro_reading);
    baro->value = baro_reading;
    Bound(baro->value, 0, 1024);
  }
  
  baro->next_update += NPS_BARO_DT;
  baro->data_available = TRUE;
}
