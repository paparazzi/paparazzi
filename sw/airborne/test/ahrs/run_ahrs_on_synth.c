#include <string.h>

#include "test/ahrs/ahrs_on_synth.h"

#include "subsystems/imu.h"
#include "subsystems/ahrs.h"
#include "subsystems/ahrs/ahrs_aligner.h"

#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "test/pprz_algebra_print.h"


static void report(void);

int main(int argc, char **argv)
{

  int traj = 0;
  if (argc > 1) {
    traj = atoi(argv[1]);
  }

  aos_init(traj);
  report();

  //  while (aos.time < 0.01) {
  while (aos.time < aos.traj->te) {
    aos_run();
    report();
  }

  return 0;
}


static void report(void)
{

  int output_sensors = FALSE;
  int output_pos = FALSE;

  printf("%f ", aos.time);

  printf("%f %f %f ", DegOfRad(aos.ltp_to_imu_euler.phi),
         DegOfRad(aos.ltp_to_imu_euler.theta),
         DegOfRad(aos.ltp_to_imu_euler.psi));

  printf("%f %f %f ", DegOfRad(aos.imu_rates.p),
         DegOfRad(aos.imu_rates.q),
         DegOfRad(aos.imu_rates.r));

  printf("%f %f %f ", DegOfRad(aos.gyro_bias.p),
         DegOfRad(aos.gyro_bias.q),
         DegOfRad(aos.gyro_bias.r));

#if AHRS_TYPE == AHRS_TYPE_ICQ
  struct Int32Eulers ltp_to_imu_euler_i;
  int32_eulers_of_quat(&ltp_to_imu_euler_i, &ahrs_impl.ltp_to_imu_quat);
  struct FloatEulers ltp_to_imu_euler_f;
  EULERS_FLOAT_OF_BFP(ltp_to_imu_euler_f, ltp_to_imu_euler_i);
  printf("%f %f %f ", DegOfRad(ltp_to_imu_euler_f.phi),
         DegOfRad(ltp_to_imu_euler_f.theta),
         DegOfRad(ltp_to_imu_euler_f.psi));

  struct FloatRates imu_rate_f;
  RATES_FLOAT_OF_BFP(imu_rate_f, ahrs_impl.imu_rate);
  printf("%f %f %f ", DegOfRad(imu_rate_f.p),
         DegOfRad(imu_rate_f.q),
         DegOfRad(imu_rate_f.r));

  struct FloatRates bias;
  RATES_FLOAT_OF_BFP(bias, ahrs_impl.gyro_bias);
  printf("%f %f %f ", DegOfRad(bias.p),
         DegOfRad(bias.q),
         DegOfRad(bias.r));

#elif AHRS_TYPE == AHRS_TYPE_FCR2 || AHRS_TYPE == AHRS_TYPE_FCQ || AHRS_TYPE == AHRS_TYPE_FCR

  printf("%f %f %f ", DegOfRad(ahrs_impl.ltp_to_imu_euler.phi),
         DegOfRad(ahrs_impl.ltp_to_imu_euler.theta),
         DegOfRad(ahrs_impl.ltp_to_imu_euler.psi));

  printf("%f %f %f ", DegOfRad(ahrs_impl.imu_rate.p),
         DegOfRad(ahrs_impl.imu_rate.q),
         DegOfRad(ahrs_impl.imu_rate.r));

  printf("%f %f %f ", DegOfRad(ahrs_impl.gyro_bias.p),
         DegOfRad(ahrs_impl.gyro_bias.q),
         DegOfRad(ahrs_impl.gyro_bias.r));
#endif

  if (output_pos) {
    printf("%f %f %f ", aos.ltp_pos.x,
           aos.ltp_pos.y,
           aos.ltp_pos.z);
  }

  if (output_sensors) {

  }

  printf("\n");

}
