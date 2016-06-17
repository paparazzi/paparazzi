#include <glib.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#include "test/ahrs/ahrs_on_synth.h"

#include "subsystems/imu.h"
#include "subsystems/ahrs.h"

#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "test/pprz_algebra_print.h"




gboolean timeout_callback(gpointer data)
{

  for (int i = 0; i < 20; i++) {
    aos_compute_state();
    aos_compute_sensors();
#ifndef DISABLE_PROPAGATE
    ahrs_propagate(aos.dt);
#endif
#ifndef DISABLE_ACCEL_UPDATE
    ahrs_update_accel();
#endif
#ifndef DISABLE_MAG_UPDATE
    if (!(i % 5)) { ahrs_update_mag(); }
#endif
  }

#if AHRS_TYPE == AHRS_TYPE_ICE || AHRS_TYPE == AHRS_TYPE_ICQ
  EULERS_FLOAT_OF_BFP(ahrs_float.ltp_to_imu_euler, ahrs.ltp_to_imu_euler);
#endif

#if AHRS_TYPE == AHRS_TYPE_ICQ
  IvySendMsg("183 AHRS_GYRO_BIAS_INT %d %d %d %d",
             ahrs_impl.gyro_bias.p,
             ahrs_impl.gyro_bias.q,
             ahrs_impl.gyro_bias.r, 1);
#endif
#if AHRS_TYPE == AHRS_TYPE_FLQ || AHRS_TYPE == AHRS_TYPE_FCR2
  struct Int32Rates bias_i;
  RATES_BFP_OF_REAL(bias_i, ahrs_impl.gyro_bias);
  IvySendMsg("183 AHRS_GYRO_BIAS_INT %d %d %d %d",
             bias_i.p,
             bias_i.q,
             bias_i.r, 1);
#endif

  IvySendMsg("183 AHRS_EULER %f %f %f %d",
             ahrs_float.ltp_to_imu_euler.phi,
             ahrs_float.ltp_to_imu_euler.theta,
             ahrs_float.ltp_to_imu_euler.psi, 1);

  IvySendMsg("183 NPS_RATE_ATTITUDE %f %f %f %f %f %f",
             DegOfRad(aos.imu_rates.p),
             DegOfRad(aos.imu_rates.q),
             DegOfRad(aos.imu_rates.r),
             DegOfRad(aos.ltp_to_imu_euler.phi),
             DegOfRad(aos.ltp_to_imu_euler.theta),
             DegOfRad(aos.ltp_to_imu_euler.psi));

  IvySendMsg("183 NPS_GYRO_BIAS %f %f %f",
             DegOfRad(aos.gyro_bias.p),
             DegOfRad(aos.gyro_bias.q),
             DegOfRad(aos.gyro_bias.r));

  return true;
}





int main(int argc, char **argv)
{

  printf("hello\n");

  g_timeout_add(1000 / 25, timeout_callback, NULL);

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);

  IvyInit("test_ahrs", "test_ahrs READY", NULL, NULL, NULL, NULL);
  IvyStart("127.255.255.255");

  imu_init();
  ahrs_init();

  aos_init();

  g_main_loop_run(ml);

  return 0;
}

