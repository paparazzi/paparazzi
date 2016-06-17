#include "nps_sensor_baro.h"

#include "generated/airframe.h"

#include "std.h"
#include "nps_fdm.h"
#include "nps_random.h"
#include NPS_SENSORS_PARAMS

#ifndef NPS_BARO_NOISE_STD_DEV
#define NPS_BARO_NOISE_STD_DEV 0.
#endif

void nps_sensor_baro_init(struct NpsSensorBaro *baro, double time)
{
  baro->value = 0.;
  baro->noise_std_dev = NPS_BARO_NOISE_STD_DEV;
  baro->next_update = time;
  baro->data_available = FALSE;
}


void nps_sensor_baro_run_step(struct NpsSensorBaro *baro, double time)
{
  if (time < baro->next_update) {
    return;
  }

  /* pressure in Pascal */
  baro->value = fdm.pressure;
  /* add noise with std dev Pascal */
  baro->value += get_gaussian_noise() * baro->noise_std_dev;

  baro->next_update += NPS_BARO_DT;
  baro->data_available = TRUE;
}
