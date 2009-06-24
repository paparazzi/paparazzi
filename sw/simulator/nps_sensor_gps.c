#include "nps_sensor_gps.h"

#include "airframe.h"
#include "nps_fdm.h"
#include "nps_random.h"
#include NPS_SENSORS_PARAMS

void nps_sensor_gps_init(struct NpsSensorGps* gps, double time) {
  FLOAT_VECT3_ZERO(gps->ecef_pos);
  FLOAT_VECT3_ZERO(gps->ecef_vel);
  gps->next_update = time;
  gps->data_available = FALSE;
}


void nps_sensor_gps_run_step(struct NpsSensorGps* gps, double time) {

  if (time < gps->next_update)
    return;

  VECT3_COPY(gps->ecef_pos, fdm.ecef_pos);
  VECT3_COPY(gps->ecef_vel, fdm.ecef_vel);

  gps->next_update += NPS_GPS_DT;
  gps->data_available = TRUE;

}

