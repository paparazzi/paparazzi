#include "nps_sensor_gps.h"

#include <stdio.h>
#include "generated/airframe.h"
#include "nps_fdm.h"
#include "nps_random.h"
#include "nps_sensors_utils.h"
#include NPS_SENSORS_PARAMS

void nps_sensor_gps_init(struct NpsSensorGps *gps, double time)
{
  FLOAT_VECT3_ZERO(gps->ecef_pos);
  FLOAT_VECT3_ZERO(gps->ecef_vel);
  gps->hmsl = 0.0;
  gps->pos_latency = NPS_GPS_POS_LATENCY;
  gps->speed_latency = NPS_GPS_SPEED_LATENCY;
  VECT3_ASSIGN(gps->pos_noise_std_dev,
               NPS_GPS_POS_NOISE_STD_DEV, NPS_GPS_POS_NOISE_STD_DEV, NPS_GPS_POS_NOISE_STD_DEV);
  VECT3_ASSIGN(gps->speed_noise_std_dev,
               NPS_GPS_SPEED_NOISE_STD_DEV, NPS_GPS_SPEED_NOISE_STD_DEV, NPS_GPS_SPEED_NOISE_STD_DEV);
  VECT3_ASSIGN(gps->pos_bias_initial,
               NPS_GPS_POS_BIAS_INITIAL_X, NPS_GPS_POS_BIAS_INITIAL_Y, NPS_GPS_POS_BIAS_INITIAL_Z);
  VECT3_ASSIGN(gps->pos_bias_random_walk_std_dev,
               NPS_GPS_POS_BIAS_RANDOM_WALK_STD_DEV_X,
               NPS_GPS_POS_BIAS_RANDOM_WALK_STD_DEV_Y,
               NPS_GPS_POS_BIAS_RANDOM_WALK_STD_DEV_Z);
  FLOAT_VECT3_ZERO(gps->pos_bias_random_walk_value);
  gps->next_update = time;
  gps->data_available = FALSE;
}

/*
 * WARNING!
 * noise and bias is currently added in ECEF frame
 * If noise or bias is not the same for all axes this has to be done in LTP!!
 *
 */

void nps_sensor_gps_run_step(struct NpsSensorGps *gps, double time)
{
  if (time < gps->next_update) {
    return;
  }

  /*
   * simulate speed sensor
   */
  struct DoubleVect3 cur_speed_reading;
  VECT3_COPY(cur_speed_reading, fdm.ecef_ecef_vel);
  /* add a gaussian noise */
  double_vect3_add_gaussian_noise(&cur_speed_reading, &gps->speed_noise_std_dev);

  /* store that for later and retrieve a previously stored data */
  UpdateSensorLatency(time, &cur_speed_reading, &gps->speed_history, gps->speed_latency, &gps->ecef_vel);


  /*
   * simulate position sensor
   */
  /* compute gps error readings */
  struct DoubleVect3 pos_error;
  VECT3_COPY(pos_error, gps->pos_bias_initial);
  /* add a gaussian noise */
  double_vect3_add_gaussian_noise(&pos_error, &gps->pos_noise_std_dev);
  /* update random walk bias and add it to error*/
  double_vect3_update_random_walk(&gps->pos_bias_random_walk_value, &gps->pos_bias_random_walk_std_dev, NPS_GPS_DT, 5.);
  VECT3_ADD(pos_error, gps->pos_bias_random_walk_value);

  /* add error to current pos reading */
  struct DoubleVect3 cur_pos_reading;
  VECT3_COPY(cur_pos_reading, fdm.ecef_pos);
  VECT3_ADD(cur_pos_reading, pos_error);

  /* store that for later and retrieve a previously stored data */
  UpdateSensorLatency(time, &cur_pos_reading, &gps->pos_history, gps->pos_latency, &gps->ecef_pos);


  /*
   * simulate lla pos
   */
  /* convert current ecef reading to lla */
  struct LlaCoor_d cur_lla_reading;
  lla_of_ecef_d(&cur_lla_reading, (struct EcefCoor_d *) &cur_pos_reading);

  /* store that for later and retrieve a previously stored data */
  UpdateSensorLatency(time, &cur_lla_reading, &gps->lla_history, gps->pos_latency, &gps->lla_pos);

  double cur_hmsl_reading = fdm.hmsl;
  UpdateSensorLatency_Single(time, &cur_hmsl_reading, &gps->hmsl_history, gps->pos_latency, &gps->hmsl);

  gps->next_update += NPS_GPS_DT;
#ifndef NPS_NO_GPS
  gps->data_available = TRUE;
#endif
}

