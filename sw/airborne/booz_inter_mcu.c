#include "booz_inter_mcu.h"

#include "imu_v3.h"
#include "booz_ahrs.h"


struct booz_inter_mcu_state inter_mcu_state;

#ifdef BOOZ_FILTER_MCU

#define LP_GYROS

void inter_mcu_fill_state() {
#ifndef LP_GYROS
  inter_mcu_state.r_rates[AXIS_P]  = imu_vs_gyro_unbiased[AXIS_P] * RATE_PI_S/M_PI;
  inter_mcu_state.r_rates[AXIS_Q]  = imu_vs_gyro_unbiased[AXIS_Q] * RATE_PI_S/M_PI;
  inter_mcu_state.r_rates[AXIS_R]  = imu_vs_gyro_unbiased[AXIS_R] * RATE_PI_S/M_PI;
#else
  inter_mcu_state.r_rates[AXIS_P]  = imu_gyro_lp[AXIS_P] * RATE_PI_S/M_PI;
  inter_mcu_state.r_rates[AXIS_Q]  = imu_gyro_lp[AXIS_Q] * RATE_PI_S/M_PI;
  inter_mcu_state.r_rates[AXIS_R]  = imu_gyro_lp[AXIS_R] * RATE_PI_S/M_PI;
#endif
  inter_mcu_state.f_rates[AXIS_P]  = booz_ahrs_p * RATE_PI_S/M_PI;
  inter_mcu_state.f_rates[AXIS_Q]  = booz_ahrs_q * RATE_PI_S/M_PI;
  inter_mcu_state.f_rates[AXIS_R]  = booz_ahrs_r * RATE_PI_S/M_PI;
  inter_mcu_state.f_eulers[AXIS_X] = booz_ahrs_phi * ANGLE_PI/M_PI;
  inter_mcu_state.f_eulers[AXIS_Y] = booz_ahrs_theta* ANGLE_PI/M_PI;
  inter_mcu_state.f_eulers[AXIS_Z] = booz_ahrs_psi  * ANGLE_PI/M_PI;
  inter_mcu_state.status =  booz_ahrs_status;
}
#endif
