#include "nps_sensor_temperature.h"

#include "generated/airframe.h"

#include "std.h"
#include "nps_fdm.h"
#include "nps_random.h"
#include NPS_SENSORS_PARAMS

/// 10Hz default
#ifndef NPS_TEMPERATURE_DT
#define NPS_TEMPERATURE_DT 0.01
#endif

#ifndef NPS_TEMPERATURE_NOISE_STD_DEV
#define NPS_TEMPERATURE_NOISE_STD_DEV 0.1
#endif

void nps_sensor_temperature_init(struct NpsSensorTemperature *temperature, double time)
{
  temperature->value = 0.;
  temperature->noise_std_dev = NPS_TEMPERATURE_NOISE_STD_DEV;
  temperature->next_update = time;
  temperature->data_available = FALSE;
}


void nps_sensor_temperature_run_step(struct NpsSensorTemperature *temperature, double time)
{
  if (time < temperature->next_update) {
    return;
  }

  /* termperature in degrees Celcius */
  temperature->value = fdm.temperature;
  /* add noise with std dev */
  temperature->value += get_gaussian_noise() * temperature->noise_std_dev;

  temperature->next_update += NPS_TEMPERATURE_DT;
  temperature->data_available = TRUE;
}
