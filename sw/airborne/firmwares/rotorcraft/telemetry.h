/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "std.h"
#include "messages.h"
#include "mcu_periph/uart.h"

#include "subsystems/datalink/downlink.h"
#include "generated/periodic_telemetry.h"

#ifdef RADIO_CONTROL
#include "subsystems/radio_control.h"
#endif

#include "state.h"

#include "firmwares/rotorcraft/autopilot.h"
#include "firmwares/rotorcraft/guidance.h"

#include "subsystems/actuators.h"

#include "mcu_periph/sys_time.h"
#include "subsystems/electrical.h"
#include "subsystems/imu.h"
#if USE_GPS
#include "subsystems/gps.h"
#endif
#include "subsystems/ins.h"
#include "subsystems/ahrs.h"
// I2C Error counters
#include "mcu_periph/i2c.h"

#define PERIODIC_SEND_ALIVE(_trans, _dev) DOWNLINK_SEND_ALIVE(_trans, _dev, 16, MD5SUM)

#if USE_GPS
#define PERIODIC_SEND_ROTORCRAFT_STATUS(_trans, _dev) {			\
    uint32_t imu_nb_err = 0;						\
    uint8_t _twi_blmc_nb_err = 0;					\
    uint16_t time_sec = sys_time.nb_sec; \
    DOWNLINK_SEND_ROTORCRAFT_STATUS(_trans, _dev,				\
                    &imu_nb_err,			\
                    &_twi_blmc_nb_err,			\
                    &radio_control.status,		\
                    &radio_control.frame_rate,		\
                    &gps.fix,		\
                    &autopilot_mode,			\
                    &autopilot_in_flight,		\
                    &autopilot_motors_on,		\
                    &guidance_h_mode,			\
                    &guidance_v_mode,			\
                    &electrical.vsupply,		\
                    &time_sec			\
                    );					\
  }
#else /* !USE_GPS */
#define PERIODIC_SEND_ROTORCRAFT_STATUS(_trans, _dev) {			\
    uint32_t imu_nb_err = 0;						\
    uint8_t twi_blmc_nb_err = 0;					\
    uint8_t  fix = GPS_FIX_NONE;					\
    uint16_t time_sec = sys_time.nb_sec;                            \
    DOWNLINK_SEND_ROTORCRAFT_STATUS(_trans, _dev,					\
                  &imu_nb_err,				\
                  &twi_blmc_nb_err,				\
                  &radio_control.status,			\
                  &radio_control.frame_rate,			\
                  &fix,					\
                  &autopilot_mode,			\
                  &autopilot_in_flight,		\
                  &autopilot_motors_on,		\
                  &guidance_h_mode,			\
                  &guidance_v_mode,			\
                  &electrical.vsupply,		\
                  &time_sec    \
                  );					\
  }
#endif /* USE_GPS */

#ifdef RADIO_CONTROL
#define PERIODIC_SEND_RC(_trans, _dev) DOWNLINK_SEND_RC(_trans, _dev, RADIO_CONTROL_NB_CHANNEL, radio_control.values)
#if defined RADIO_KILL_SWITCH
#define PERIODIC_SEND_ROTORCRAFT_RADIO_CONTROL(_trans, _dev) SEND_ROTORCRAFT_RADIO_CONTROL( _trans, _dev, &radio_control.values[RADIO_KILL_SWITCH])
#else /* ! RADIO_KILL_SWITCH */
#define PERIODIC_SEND_ROTORCRAFT_RADIO_CONTROL(_trans, _dev) {                              \
    int16_t foo = -42;                                              \
    SEND_ROTORCRAFT_RADIO_CONTROL( _trans, _dev, &foo)                                  \
}
#endif /* !RADIO_KILL_SWITCH */
#define SEND_ROTORCRAFT_RADIO_CONTROL(_trans, _dev, _kill_switch) {                             \
    DOWNLINK_SEND_ROTORCRAFT_RADIO_CONTROL(_trans, _dev,                                    \
                      &radio_control.values[RADIO_ROLL],            \
                      &radio_control.values[RADIO_PITCH],           \
                      &radio_control.values[RADIO_YAW],             \
                      &radio_control.values[RADIO_THROTTLE],        \
                      &radio_control.values[RADIO_MODE],            \
                      _kill_switch,                                        \
                      &radio_control.status);}
#else /* ! RADIO_CONTROL */
#define PERIODIC_SEND_RC(_trans, _dev) {}
#define PERIODIC_SEND_ROTORCRAFT_RADIO_CONTROL(_trans, _dev) {}
#endif

#ifdef RADIO_CONTROL_TYPE_PPM
#define PERIODIC_SEND_PPM(_trans, _dev) {                             \
    uint16_t ppm_pulses_usec[RADIO_CONTROL_NB_CHANNEL];        \
    for (int i=0;i<RADIO_CONTROL_NB_CHANNEL;i++)               \
      ppm_pulses_usec[i] = USEC_OF_RC_PPM_TICKS(ppm_pulses[i]); \
    DOWNLINK_SEND_PPM(_trans, _dev,                                   \
                      &radio_control.frame_rate,               \
                      PPM_NB_CHANNEL,                          \
                      ppm_pulses_usec);                        \
  }
#else
#define PERIODIC_SEND_PPM(_trans, _dev) {}
#endif

#ifdef ACTUATORS
#define PERIODIC_SEND_ACTUATORS(_trans, _dev) DOWNLINK_SEND_ACTUATORS(_trans, _dev, ACTUATORS_NB, actuators)
#else
#define PERIODIC_SEND_ACTUATORS(_trans, _dev) {}
#endif

#define PERIODIC_SEND_IMU_GYRO_SCALED(_trans, _dev) {		\
    DOWNLINK_SEND_IMU_GYRO_SCALED(_trans, _dev,			\
                 &imu.gyro.p,		\
                 &imu.gyro.q,		\
                 &imu.gyro.r);		\
  }

#define PERIODIC_SEND_IMU_ACCEL_SCALED(_trans, _dev) {			\
    DOWNLINK_SEND_IMU_ACCEL_SCALED(_trans, _dev,				\
                  &imu.accel.x,		\
                  &imu.accel.y,		\
                  &imu.accel.z);		\
  }

#define PERIODIC_SEND_IMU_MAG_SCALED(_trans, _dev) {			\
    DOWNLINK_SEND_IMU_MAG_SCALED(_trans, _dev,				\
                &imu.mag.x,			\
                &imu.mag.y,			\
                &imu.mag.z);			\
  }

#define PERIODIC_SEND_IMU_GYRO_RAW(_trans, _dev) {				\
    DOWNLINK_SEND_IMU_GYRO_RAW(_trans, _dev,					\
                   &imu.gyro_unscaled.p,		\
                   &imu.gyro_unscaled.q,		\
                   &imu.gyro_unscaled.r);		\
  }

#define PERIODIC_SEND_IMU_ACCEL_RAW(_trans, _dev) {				\
    DOWNLINK_SEND_IMU_ACCEL_RAW(_trans, _dev,					\
                &imu.accel_unscaled.x,		\
                &imu.accel_unscaled.y,		\
                &imu.accel_unscaled.z);		\
  }

#define PERIODIC_SEND_IMU_MAG_RAW(_trans, _dev) {				\
    DOWNLINK_SEND_IMU_MAG_RAW(_trans, _dev,					\
                  &imu.mag_unscaled.x,			\
                  &imu.mag_unscaled.y,			\
                  &imu.mag_unscaled.z);		\
  }

#define PERIODIC_SEND_IMU_MAG_CURRENT_CALIBRATION(_trans, _dev) {                               \
    DOWNLINK_SEND_IMU_MAG_CURRENT_CALIBRATION(_trans, _dev,                                     \
                  &imu.mag_unscaled.x,                  \
                  &imu.mag_unscaled.y,                  \
                  &imu.mag_unscaled.z,                  \
                  &electrical.current);               \
  }

#include "subsystems/sensors/baro.h"
#define PERIODIC_SEND_BARO_RAW(_trans, _dev) {         \
    DOWNLINK_SEND_BARO_RAW(_trans, _dev,               \
                           &baro.absolute,      \
                           &baro.differential); \
  }



#include "firmwares/rotorcraft/stabilization.h"
#define PERIODIC_SEND_RATE_LOOP(_trans, _dev) {                          \
    DOWNLINK_SEND_RATE_LOOP(_trans, _dev,                                \
                                  &stabilization_rate_sp.p,        \
                                  &stabilization_rate_sp.q,        \
                                  &stabilization_rate_sp.r,        \
                                  &stabilization_rate_ref.p,       \
                                  &stabilization_rate_ref.q,       \
                                  &stabilization_rate_ref.r,       \
                                  &stabilization_rate_refdot.p,    \
                                  &stabilization_rate_refdot.q,    \
                                  &stabilization_rate_refdot.r,    \
                                  &stabilization_rate_sum_err.p,    \
                                  &stabilization_rate_sum_err.q,    \
                                  &stabilization_rate_sum_err.r,    \
                                  &stabilization_rate_ff_cmd.p,    \
                                  &stabilization_rate_ff_cmd.q,    \
                                  &stabilization_rate_ff_cmd.r,    \
                                  &stabilization_rate_fb_cmd.p,    \
                                  &stabilization_rate_fb_cmd.q,    \
                                  &stabilization_rate_fb_cmd.r,    \
                                  &stabilization_cmd[COMMAND_THRUST]); \
  }


#ifdef STABILIZATION_ATTITUDE_NO_REF
#define PERIODIC_SEND_STAB_ATTITUDE(_trans, _dev) {}
#define PERIODIC_SEND_STAB_ATTITUDE_REF(_trans, _dev) {}

#elif defined STABILIZATION_ATTITUDE_TYPE_INT

#define PERIODIC_SEND_STAB_ATTITUDE(_trans, _dev) {       \
  struct Int32Rates* body_rate = stateGetBodyRates_i();   \
  struct Int32Eulers* att = stateGetNedToBodyEulers_i();  \
  DOWNLINK_SEND_STAB_ATTITUDE_INT(_trans, _dev,           \
      &(body_rate->p), &(body_rate->q), &(body_rate->r),  \
      &(att->phi), &(att->theta), &(att->psi),            \
      &stab_att_sp_euler.phi,                             \
      &stab_att_sp_euler.theta,                           \
      &stab_att_sp_euler.psi,                             \
      &stabilization_att_sum_err.phi,                     \
      &stabilization_att_sum_err.theta,                   \
      &stabilization_att_sum_err.psi,                     \
      &stabilization_att_fb_cmd[COMMAND_ROLL],            \
      &stabilization_att_fb_cmd[COMMAND_PITCH],           \
      &stabilization_att_fb_cmd[COMMAND_YAW],             \
      &stabilization_att_ff_cmd[COMMAND_ROLL],            \
      &stabilization_att_ff_cmd[COMMAND_PITCH],           \
      &stabilization_att_ff_cmd[COMMAND_YAW],             \
      &stabilization_cmd[COMMAND_ROLL],                   \
      &stabilization_cmd[COMMAND_PITCH],                  \
      &stabilization_cmd[COMMAND_YAW]);                   \
}

#define PERIODIC_SEND_STAB_ATTITUDE_REF(_trans, _dev) {             \
    DOWNLINK_SEND_STAB_ATTITUDE_REF_INT(_trans, _dev,               \
                                        &stab_att_sp_euler.phi,     \
                                        &stab_att_sp_euler.theta,   \
                                        &stab_att_sp_euler.psi,     \
                                        &stab_att_ref_euler.phi,    \
                                        &stab_att_ref_euler.theta,  \
                                        &stab_att_ref_euler.psi,    \
                                        &stab_att_ref_rate.p,       \
                                        &stab_att_ref_rate.q,       \
                                        &stab_att_ref_rate.r,       \
                                        &stab_att_ref_accel.p,      \
                                        &stab_att_ref_accel.q,      \
                                        &stab_att_ref_accel.r);     \
  }

#elif defined STABILIZATION_ATTITUDE_TYPE_FLOAT

#define PERIODIC_SEND_STAB_ATTITUDE(_trans, _dev) {       \
  struct FloatRates* body_rate = stateGetBodyRates_f();   \
  struct FloatEulers* att = stateGetNedToBodyEulers_f();  \
  float foo;                                              \
  DOWNLINK_SEND_STAB_ATTITUDE_FLOAT(_trans, _dev,         \
      &(body_rate->p), &(body_rate->q), &(body_rate->r),  \
      &(att->phi), &(att->theta), &(att->psi),            \
      &stab_att_ref_euler.phi,                            \
      &stab_att_ref_euler.theta,                          \
      &stab_att_ref_euler.psi,                            \
      &stabilization_att_sum_err_eulers.phi,              \
      &stabilization_att_sum_err_eulers.theta,            \
      &stabilization_att_sum_err_eulers.psi,              \
      &stabilization_att_fb_cmd[COMMAND_ROLL],            \
      &stabilization_att_fb_cmd[COMMAND_PITCH],           \
      &stabilization_att_fb_cmd[COMMAND_YAW],             \
      &stabilization_att_ff_cmd[COMMAND_ROLL],            \
      &stabilization_att_ff_cmd[COMMAND_PITCH],           \
      &stabilization_att_ff_cmd[COMMAND_YAW],             \
      &stabilization_cmd[COMMAND_ROLL],                   \
      &stabilization_cmd[COMMAND_PITCH],                  \
      &stabilization_cmd[COMMAND_YAW],                    \
      &foo, &foo, &foo);                                  \
  }

#define PERIODIC_SEND_STAB_ATTITUDE_REF(_trans, _dev) {                 \
    DOWNLINK_SEND_STAB_ATTITUDE_REF_FLOAT(_trans, _dev,                 \
                                          &stab_att_sp_euler.phi,       \
                                          &stab_att_sp_euler.theta,     \
                                          &stab_att_sp_euler.psi,       \
                                          &stab_att_ref_euler.phi,      \
                                          &stab_att_ref_euler.theta,    \
                                          &stab_att_ref_euler.psi,      \
                                          &stab_att_ref_rate.p,         \
                                          &stab_att_ref_rate.q,         \
                                          &stab_att_ref_rate.r,         \
                                          &stab_att_ref_accel.p,        \
                                          &stab_att_ref_accel.q,        \
                                          &stab_att_ref_accel.r);       \
  }

#endif /* STABILIZATION_ATTITUDE_TYPE_FLOAT */


#include "subsystems/ahrs/ahrs_aligner.h"
#define PERIODIC_SEND_FILTER_ALIGNER(_trans, _dev) {			\
    DOWNLINK_SEND_FILTER_ALIGNER(_trans, _dev,                  \
                                 &ahrs_aligner.lp_gyro.p,       \
                                 &ahrs_aligner.lp_gyro.q,       \
                                 &ahrs_aligner.lp_gyro.r,       \
                                 &imu.gyro.p,                   \
                                 &imu.gyro.q,                   \
                                 &imu.gyro.r,                   \
                                 &ahrs_aligner.noise,           \
                                 &ahrs_aligner.low_noise_cnt,   \
                                 &ahrs_aligner.status);         \
  }


#define PERIODIC_SEND_ROTORCRAFT_CMD(_trans, _dev) {                    \
    DOWNLINK_SEND_ROTORCRAFT_CMD(_trans, _dev,                          \
                                 &stabilization_cmd[COMMAND_ROLL],      \
                                 &stabilization_cmd[COMMAND_PITCH],     \
                                 &stabilization_cmd[COMMAND_YAW],       \
                                 &stabilization_cmd[COMMAND_THRUST]);   \
  }


#if USE_AHRS_CMPL_EULER
#include "subsystems/ahrs/ahrs_int_cmpl_euler.h"
#define PERIODIC_SEND_FILTER(_trans, _dev) {					\
    DOWNLINK_SEND_FILTER(_trans, _dev,						\
             &ahrs_impl.ltp_to_imu_euler.phi,			\
             &ahrs_impl.ltp_to_imu_euler.theta,			\
             &ahrs_impl.ltp_to_imu_euler.psi,			\
             &ahrs_impl.measure.phi,			\
             &ahrs_impl.measure.theta,			\
             &ahrs_impl.measure.psi,			\
             &ahrs_impl.hi_res_euler.phi,			\
             &ahrs_impl.hi_res_euler.theta,			\
             &ahrs_impl.hi_res_euler.psi,			\
             &ahrs_impl.residual.phi,			\
             &ahrs_impl.residual.theta,			\
             &ahrs_impl.residual.psi,			\
             &ahrs_impl.gyro_bias.p,			\
             &ahrs_impl.gyro_bias.q,			\
             &ahrs_impl.gyro_bias.r);			\
  }
#else
#define PERIODIC_SEND_FILTER(_trans, _dev) {}
#endif

#if USE_AHRS_ARDRONE2
#include "subsystems/ahrs/ahrs_ardrone2.h"
#define PERIODIC_SEND_AHRS_ARDRONE2(_trans, _dev) {	\
    DOWNLINK_SEND_AHRS_ARDRONE2(_trans, _dev,	\
             &ahrs_impl.state,					\
             &ahrs_impl.control_state,			\
             &ahrs_impl.eulers.phi,				\
             &ahrs_impl.eulers.theta,			\
             &ahrs_impl.eulers.psi,				\
             &ahrs_impl.speed.x,				\
             &ahrs_impl.speed.y,				\
             &ahrs_impl.speed.z,				\
             &ahrs_impl.accel.x,				\
             &ahrs_impl.accel.y,				\
             &ahrs_impl.accel.z,				\
             &ahrs_impl.altitude,				\
             &ahrs_impl.battery);				\
  }
#else
#define PERIODIC_SEND_AHRS_ARDRONE2(_trans, _dev) {}
#endif

#if USE_AHRS_CMPL_EULER || USE_AHRS_CMPL_QUAT
#define PERIODIC_SEND_AHRS_GYRO_BIAS_INT(_trans, _dev) {    \
  DOWNLINK_SEND_AHRS_GYRO_BIAS_INT(_trans, _dev,            \
                                   &ahrs_impl.gyro_bias.p,  \
                                   &ahrs_impl.gyro_bias.q,  \
                                   &ahrs_impl.gyro_bias.r); \
  }
#else
#define PERIODIC_SEND_AHRS_GYRO_BIAS_INT(_trans, _dev) {}
#endif

#if USE_AHRS_LKF
#include "subsystems/ahrs.h"
#include "ahrs/ahrs_float_lkf.h"
#define PERIODIC_SEND_AHRS_LKF(_trans, _dev) {				\
    DOWNLINK_SEND_AHRS_LKF(&bafl_eulers.phi,			\
                _trans, _dev,					\
                &bafl_eulers.theta,			\
                &bafl_eulers.psi,			\
                &bafl_quat.qi,				\
                &bafl_quat.qx,				\
                &bafl_quat.qy,				\
                &bafl_quat.qz,				\
                &bafl_rates.p,				\
                &bafl_rates.q,				\
                &bafl_rates.r,				\
                &bafl_accel_measure.x,			\
                &bafl_accel_measure.y,			\
                &bafl_accel_measure.z,			\
                &bafl_mag.x,				\
                &bafl_mag.y,				\
                &bafl_mag.z);				\
  }
#define PERIODIC_SEND_AHRS_LKF_DEBUG(_trans, _dev) {           \
    DOWNLINK_SEND_AHRS_LKF_DEBUG(_trans, _dev,             \
                      &bafl_X[0],          \
                      &bafl_X[1],          \
                      &bafl_X[2],          \
                      &bafl_bias.p,        \
                      &bafl_bias.q,        \
                      &bafl_bias.r,        \
                      &bafl_qnorm,         \
                      &bafl_phi_accel,         \
                      &bafl_theta_accel,       \
                      &bafl_P[0][0],           \
                      &bafl_P[1][1],           \
                      &bafl_P[2][2],           \
                      &bafl_P[3][3],           \
                      &bafl_P[4][4],           \
                      &bafl_P[5][5]);          \
  }
#define PERIODIC_SEND_AHRS_LKF_ACC_DBG(_trans, _dev) {          \
    DOWNLINK_SEND_AHRS_LKF_ACC_DBG(_trans, _dev,                \
                    &bafl_q_a_err.qi,       \
                    &bafl_q_a_err.qx,       \
                    &bafl_q_a_err.qy,       \
                    &bafl_q_a_err.qz,       \
                    &bafl_b_a_err.p,        \
                    &bafl_b_a_err.q,        \
                    &bafl_b_a_err.r);       \
  }
#define PERIODIC_SEND_AHRS_LKF_MAG_DBG(_trans, _dev) {      \
    DOWNLINK_SEND_AHRS_LKF_MAG_DBG(_trans, _dev,            \
                    &bafl_q_m_err.qi,   \
                    &bafl_q_m_err.qx,   \
                    &bafl_q_m_err.qy,   \
                    &bafl_q_m_err.qz,   \
                    &bafl_b_m_err.p,    \
                    &bafl_b_m_err.q,    \
                    &bafl_b_m_err.r);   \
  }
#else
#define PERIODIC_SEND_AHRS_LKF(_trans, _dev) {}
#define PERIODIC_SEND_AHRS_LKF_DEBUG(_trans, _dev) {}
#define PERIODIC_SEND_AHRS_LKF_MAG_DBG(_trans, _dev) {}
#define PERIODIC_SEND_AHRS_LKF_ACC_DBG(_trans, _dev) {}
#endif

#if defined STABILIZATION_ATTITUDE_TYPE_QUAT && defined STABILIZATION_ATTITUDE_TYPE_INT
#define PERIODIC_SEND_AHRS_REF_QUAT(_trans, _dev) {   \
    DOWNLINK_SEND_AHRS_REF_QUAT(_trans, _dev,         \
                  &stab_att_ref_quat.qi,              \
                  &stab_att_ref_quat.qx,              \
                  &stab_att_ref_quat.qy,              \
                  &stab_att_ref_quat.qz,              \
                  &(stateGetNedToBodyQuat_i()->qi),   \
                  &(stateGetNedToBodyQuat_i()->qx),   \
                  &(stateGetNedToBodyQuat_i()->qy),   \
                  &(stateGetNedToBodyQuat_i()->qz));  \
  }
#else
#define PERIODIC_SEND_AHRS_REF_QUAT(_trans, _dev) {}
#endif /* STABILIZATION_ATTITUDE_TYPE_QUAT */

#ifndef AHRS_FLOAT
#define PERIODIC_SEND_AHRS_QUAT_INT(_trans, _dev) {   \
    DOWNLINK_SEND_AHRS_QUAT_INT(_trans, _dev,         \
                  &ahrs_impl.ltp_to_imu_quat.qi,      \
                  &ahrs_impl.ltp_to_imu_quat.qx,      \
                  &ahrs_impl.ltp_to_imu_quat.qy,      \
                  &ahrs_impl.ltp_to_imu_quat.qz,      \
                  &(stateGetNedToBodyQuat_i()->qi),   \
                  &(stateGetNedToBodyQuat_i()->qx),   \
                  &(stateGetNedToBodyQuat_i()->qy),   \
                  &(stateGetNedToBodyQuat_i()->qz));  \
  }
#endif

#if USE_AHRS_CMPL_EULER
#define PERIODIC_SEND_AHRS_EULER_INT(_trans, _dev) {      \
    DOWNLINK_SEND_AHRS_EULER_INT(_trans, _dev,            \
                   &ahrs_impl.ltp_to_imu_euler.phi,       \
                   &ahrs_impl.ltp_to_imu_euler.theta,     \
                   &ahrs_impl.ltp_to_imu_euler.psi,       \
                   &(stateGetNedToBodyEulers_i()->phi),   \
                   &(stateGetNedToBodyEulers_i()->theta), \
                   &(stateGetNedToBodyEulers_i()->psi));  \
  }
#else
#ifndef AHRS_FLOAT
#define PERIODIC_SEND_AHRS_EULER_INT(_trans, _dev) {                    \
    struct Int32Eulers ltp_to_imu_euler;                                \
    INT32_EULERS_OF_QUAT(ltp_to_imu_euler, ahrs_impl.ltp_to_imu_quat);  \
    DOWNLINK_SEND_AHRS_EULER_INT(_trans, _dev,                          \
                                 &ltp_to_imu_euler.phi,                 \
                                 &ltp_to_imu_euler.theta,               \
                                 &ltp_to_imu_euler.psi,                 \
                                 &(stateGetNedToBodyEulers_i()->phi),   \
                                 &(stateGetNedToBodyEulers_i()->theta), \
                                 &(stateGetNedToBodyEulers_i()->psi));  \
}
#else
#define PERIODIC_SEND_AHRS_EULER_INT(_trans, _dev) {                    \
    struct FloatEulers ltp_to_imu_euler;                                \
    FLOAT_EULERS_OF_QUAT(ltp_to_imu_euler, ahrs_impl.ltp_to_imu_quat);  \
    struct Int32Eulers euler_i;                                         \
    EULERS_BFP_OF_REAL(euler_i, ltp_to_imu_euler);                \
    DOWNLINK_SEND_AHRS_EULER_INT(_trans, _dev,                          \
                                 &euler_i.phi,                          \
                                 &euler_i.theta,                        \
                                 &euler_i.psi,                          \
                                 &(stateGetNedToBodyEulers_i()->phi),   \
                                 &(stateGetNedToBodyEulers_i()->theta), \
                                 &(stateGetNedToBodyEulers_i()->psi));  \
  }
#endif
#endif

#define PERIODIC_SEND_AHRS_RMAT_INT(_trans, _dev) {       \
  struct Int32RMat* att_rmat = stateGetNedToBodyRMat_i(); \
  DOWNLINK_SEND_AHRS_RMAT(_trans, _dev,                   \
      &ahrs_impl.ltp_to_imu_rmat.m[0],                    \
      &ahrs_impl.ltp_to_imu_rmat.m[1],                    \
      &ahrs_impl.ltp_to_imu_rmat.m[2],                    \
      &ahrs_impl.ltp_to_imu_rmat.m[3],                    \
      &ahrs_impl.ltp_to_imu_rmat.m[4],                    \
      &ahrs_impl.ltp_to_imu_rmat.m[5],                    \
      &ahrs_impl.ltp_to_imu_rmat.m[6],                    \
      &ahrs_impl.ltp_to_imu_rmat.m[7],                    \
      &ahrs_impl.ltp_to_imu_rmat.m[8],                    \
      &(att_rmat->m[0]),                                  \
      &(att_rmat->m[1]),                                  \
      &(att_rmat->m[2]),                                  \
      &(att_rmat->m[3]),                                  \
      &(att_rmat->m[4]),                                  \
      &(att_rmat->m[5]),                                  \
      &(att_rmat->m[6]),                                  \
      &(att_rmat->m[7]),                                  \
      &(att_rmat->m[8]));                                 \
}



#if USE_VFF
#include "subsystems/ins/vf_float.h"
#define PERIODIC_SEND_VFF(_trans, _dev) {		\
    DOWNLINK_SEND_VFF(_trans, _dev,			\
                &vff_z_meas,		\
                &vff_z,			\
                &vff_zdot,		\
                &vff_bias,		\
                & vff_P[0][0],		\
                & vff_P[1][1],		\
                & vff_P[2][2]);		\
  }
#else
#define PERIODIC_SEND_VFF(_trans, _dev) {}
#endif

#if USE_VFF_EXTENDED
#include "subsystems/ins/vf_extended_float.h"
#define PERIODIC_SEND_VFF_EXTENDED(_trans, _dev) {		\
    DOWNLINK_SEND_VFF_EXTENDED(_trans, _dev,			\
                &vff_z_meas,		\
                &vff_z_meas_baro, \
                &vff_z,			\
                &vff_zdot,		\
                &vff_bias,		\
                &vff_offset);		\
  }
#else
#define PERIODIC_SEND_VFF_EXTENDED(_trans, _dev) {}
#endif

#if USE_HFF
#include  "subsystems/ins/hf_float.h"
#define PERIODIC_SEND_HFF(_trans, _dev) {	\
    DOWNLINK_SEND_HFF(_trans, _dev,		\
                            &b2_hff_state.x,			\
                            &b2_hff_state.y,			\
                            &b2_hff_state.xdot,         \
                            &b2_hff_state.ydot,			\
                            &b2_hff_state.xdotdot,      \
                            &b2_hff_state.ydotdot);     \
  }
#define PERIODIC_SEND_HFF_DBG(_trans, _dev) {                \
    DOWNLINK_SEND_HFF_DBG(_trans, _dev,                      \
                                &b2_hff_x_meas,             \
                                &b2_hff_y_meas,             \
                                &b2_hff_xd_meas,            \
                                &b2_hff_yd_meas,            \
                                &b2_hff_state.xP[0][0],     \
                                &b2_hff_state.yP[0][0],     \
                                &b2_hff_state.xP[1][1],     \
                                &b2_hff_state.yP[1][1]);    \
  }
#ifdef GPS_LAG
#define PERIODIC_SEND_HFF_GPS(_trans, _dev) {               \
    DOWNLINK_SEND_HFF_GPS(_trans, _dev,                     \
                          &(b2_hff_rb_last->lag_counter),   \
                          &lag_counter_err,                 \
                          &save_counter);                   \
  }
#else
#define PERIODIC_SEND_HFF_GPS(_trans, _dev) {}
#endif
#else
#define PERIODIC_SEND_HFF(_trans, _dev) {}
#define PERIODIC_SEND_HFF_DBG(_trans, _dev) {}
#define PERIODIC_SEND_HFF_GPS(_trans, _dev) {}
#endif

#define PERIODIC_SEND_GUIDANCE_H_INT(_trans, _dev) {   \
  DOWNLINK_SEND_GUIDANCE_H_INT(_trans, _dev,           \
                               &guidance_h_pos_sp.x,   \
                               &guidance_h_pos_sp.y,   \
                               &guidance_h_pos_ref.x,  \
                               &guidance_h_pos_ref.y,  \
                               &ins_ltp_pos.x,         \
                               &ins_ltp_pos.y);        \
  }

#define PERIODIC_SEND_INS_Z(_trans, _dev) {				\
    DOWNLINK_SEND_INS_Z(_trans, _dev,					\
                &ins_baro_alt,				\
                &ins_ltp_pos.z,			\
                &ins_ltp_speed.z,			\
                &ins_ltp_accel.z);			\
  }

#define PERIODIC_SEND_INS(_trans, _dev) {			\
    DOWNLINK_SEND_INS(_trans, _dev,				\
                       &ins_ltp_pos.x,		\
                       &ins_ltp_pos.y,      \
                       &ins_ltp_pos.z,		\
                       &ins_ltp_speed.x,	\
                       &ins_ltp_speed.y,	\
                       &ins_ltp_speed.z,	\
                       &ins_ltp_accel.x,	\
                       &ins_ltp_accel.y,	\
                       &ins_ltp_accel.z);	\
  }

#define PERIODIC_SEND_INS_REF(_trans, _dev) {       \
    if (ins_ltp_initialised)                        \
      DOWNLINK_SEND_INS_REF(_trans, _dev,           \
                            &ins_ltp_def.ecef.x,    \
                            &ins_ltp_def.ecef.y,    \
                            &ins_ltp_def.ecef.z,    \
                            &ins_ltp_def.lla.lat,   \
                            &ins_ltp_def.lla.lon,   \
                            &ins_ltp_def.lla.alt,   \
                            &ins_ltp_def.hmsl,		\
                            &ins_qfe);				\
  }

#define PERIODIC_SEND_VERT_LOOP(_trans, _dev) {				\
    DOWNLINK_SEND_VERT_LOOP(_trans, _dev,				\
                  &guidance_v_z_sp,		\
                  &guidance_v_zd_sp,		\
                  &ins_ltp_pos.z,			\
                  &ins_ltp_speed.z,		\
                  &ins_ltp_accel.z,		\
                  &guidance_v_z_ref,		\
                  &guidance_v_zd_ref,		\
                  &guidance_v_zdd_ref,		\
                  &gv_adapt_X,			\
                  &gv_adapt_P,			\
                  &gv_adapt_Xmeas,			\
                  &guidance_v_z_sum_err,		\
                  &guidance_v_ff_cmd,		\
                  &guidance_v_fb_cmd,		\
                  &guidance_v_delta_t);		\
  }

#define PERIODIC_SEND_HOVER_LOOP(_trans, _dev) {				\
    DOWNLINK_SEND_HOVER_LOOP(_trans, _dev,				\
                   &guidance_h_pos_sp.x,		\
                   &guidance_h_pos_sp.y,		\
                   &ins_ltp_pos.x,			\
                   &ins_ltp_pos.y,			\
                   &ins_ltp_speed.x,		\
                   &ins_ltp_speed.y,		\
                   &ins_ltp_accel.x,		\
                   &ins_ltp_accel.y,		\
                   &guidance_h_pos_err.x,		\
                   &guidance_h_pos_err.y,		\
                   &guidance_h_speed_err.x,	\
                   &guidance_h_speed_err.y,	\
                   &guidance_h_pos_err_sum.x,	\
                   &guidance_h_pos_err_sum.y,	\
                   &guidance_h_nav_err.x,	\
                   &guidance_h_nav_err.y,	\
                   &guidance_h_command_earth.x,	\
                   &guidance_h_command_earth.y,	\
                   &guidance_h_command_body.phi,	\
                   &guidance_h_command_body.theta, \
                   &guidance_h_command_body.psi);	\
  }

#define PERIODIC_SEND_GUIDANCE_H_REF(_trans, _dev) { \
  DOWNLINK_SEND_GUIDANCE_H_REF_INT(_trans, _dev, \
      &guidance_h_pos_sp.x, \
      &guidance_h_pos_ref.x, \
      &guidance_h_speed_ref.x, \
      &guidance_h_accel_ref.x, \
      &guidance_h_pos_sp.y, \
      &guidance_h_pos_ref.y, \
      &guidance_h_speed_ref.y, \
      &guidance_h_accel_ref.y); \
}

#include "firmwares/rotorcraft/navigation.h"
#define PERIODIC_SEND_ROTORCRAFT_FP(_trans, _dev) {					\
    int32_t carrot_up = -guidance_v_z_sp;				\
    DOWNLINK_SEND_ROTORCRAFT_FP( _trans, _dev,					\
                                 &(stateGetPositionEnu_i()->x),         \
                                 &(stateGetPositionEnu_i()->y),         \
                                 &(stateGetPositionEnu_i()->z),         \
                                 &(stateGetSpeedEnu_i()->x),            \
                                 &(stateGetSpeedEnu_i()->y),            \
                                 &(stateGetSpeedEnu_i()->z),            \
                                 &(stateGetNedToBodyEulers_i()->phi),   \
                                 &(stateGetNedToBodyEulers_i()->theta), \
                                 &(stateGetNedToBodyEulers_i()->psi),   \
                                 &guidance_h_pos_sp.y,                  \
                                 &guidance_h_pos_sp.x,                  \
                                 &carrot_up,                            \
                                 &guidance_h_command_body.psi,          \
                                 &stabilization_cmd[COMMAND_THRUST],    \
                                 &autopilot_flight_time);               \
  }

#if USE_GPS
#define PERIODIC_SEND_GPS_INT(_trans, _dev) {				\
    DOWNLINK_SEND_GPS_INT( _trans, _dev,                   \
                           &gps.ecef_pos.x,         \
                           &gps.ecef_pos.y,         \
                           &gps.ecef_pos.z,         \
                           &gps.lla_pos.lat,        \
                           &gps.lla_pos.lon,        \
                           &gps.lla_pos.alt,        \
                           &gps.hmsl,               \
                           &gps.ecef_vel.x,         \
                           &gps.ecef_vel.y,         \
                           &gps.ecef_vel.z,         \
                           &gps.pacc,               \
                           &gps.sacc,               \
                           &gps.tow,                \
                           &gps.pdop,               \
                           &gps.num_sv,             \
                           &gps.fix);               \
    static uint8_t i;                               \
    static uint8_t last_cnos[GPS_NB_CHANNELS];      \
    if (i == gps.nb_channels) i = 0;                \
    if (i < gps.nb_channels && gps.svinfos[i].cno > 0 && gps.svinfos[i].cno != last_cnos[i]) { \
      DOWNLINK_SEND_SVINFO(DefaultChannel, DefaultDevice, &i, &gps.svinfos[i].svid, &gps.svinfos[i].flags, &gps.svinfos[i].qi, &gps.svinfos[i].cno, &gps.svinfos[i].elev, &gps.svinfos[i].azim); \
      last_cnos[i] = gps.svinfos[i].cno;                                \
    }                                                                   \
    i++;                                                                \
  }
#else
#define PERIODIC_SEND_GPS_INT(_trans, _dev) {}
#endif

#include "firmwares/rotorcraft/navigation.h"
#define PERIODIC_SEND_ROTORCRAFT_NAV_STATUS(_trans, _dev) {				\
    DOWNLINK_SEND_ROTORCRAFT_NAV_STATUS(_trans, _dev,                      \
                   &block_time,				\
                   &stage_time,				\
                   &nav_block,				\
                   &nav_stage,				\
                   &horizontal_mode);			\
    if (horizontal_mode == HORIZONTAL_MODE_ROUTE) {			\
      float sx = POS_FLOAT_OF_BFP(waypoints[nav_segment_start].x);	\
      float sy = POS_FLOAT_OF_BFP(waypoints[nav_segment_start].y);	\
      float ex = POS_FLOAT_OF_BFP(waypoints[nav_segment_end].x);	\
      float ey = POS_FLOAT_OF_BFP(waypoints[nav_segment_end].y);	\
      DOWNLINK_SEND_SEGMENT(_trans, _dev, &sx, &sy, &ex, &ey);			\
    }									\
    else if (horizontal_mode == HORIZONTAL_MODE_CIRCLE) {			\
      float cx = POS_FLOAT_OF_BFP(waypoints[nav_circle_centre].x);	\
      float cy = POS_FLOAT_OF_BFP(waypoints[nav_circle_centre].y);	\
      float r = POS_FLOAT_OF_BFP(nav_circle_radius); \
      DOWNLINK_SEND_CIRCLE(_trans, _dev, &cx, &cy, &r);			\
    }									\
  }

#define PERIODIC_SEND_WP_MOVED(_trans, _dev) {					\
    static uint8_t i;							\
    i++; if (i >= nb_waypoint) i = 0;					\
    DOWNLINK_SEND_WP_MOVED_ENU(_trans, _dev,					\
                   &i,					\
                   &(waypoints[i].x),			\
                   &(waypoints[i].y),			\
                   &(waypoints[i].z));			\
  }

#if USE_CAM
#define PERIODIC_SEND_ROTORCRAFT_CAM(_trans, _dev) DOWNLINK_SEND_ROTORCRAFT_CAM(_trans, _dev,&rotorcraft_cam_tilt,&rotorcraft_cam_pan);
#else
#define PERIODIC_SEND_ROTORCRAFT_CAM(_trans, _dev) {}
#endif

#ifndef AHRS_FLOAT
#define PERIODIC_SEND_ROTORCRAFT_TUNE_HOVER(_trans, _dev) {             \
    DOWNLINK_SEND_ROTORCRAFT_TUNE_HOVER(_trans, _dev,                   \
                                        &radio_control.values[RADIO_ROLL], \
                                        &radio_control.values[RADIO_PITCH], \
                                        &radio_control.values[RADIO_YAW], \
                                        &stabilization_cmd[COMMAND_ROLL], \
                                        &stabilization_cmd[COMMAND_PITCH], \
                                        &stabilization_cmd[COMMAND_YAW], \
                                        &stabilization_cmd[COMMAND_THRUST], \
                                        &ahrs_impl.ltp_to_imu_euler.phi,     \
                                        &ahrs_impl.ltp_to_imu_euler.theta,   \
                                        &ahrs_impl.ltp_to_imu_euler.psi,     \
                                        &(stateGetNedToBodyEulers_i()->phi),    \
                                        &(stateGetNedToBodyEulers_i()->theta),  \
                                        &(stateGetNedToBodyEulers_i()->psi));   \
  }
#endif

#ifdef USE_I2C0
#define PERIODIC_SEND_I2C0_ERRORS(_trans, _dev) {                             \
    uint16_t i2c0_ack_fail_cnt          = i2c0.errors->ack_fail_cnt;          \
    uint16_t i2c0_miss_start_stop_cnt   = i2c0.errors->miss_start_stop_cnt;   \
    uint16_t i2c0_arb_lost_cnt          = i2c0.errors->arb_lost_cnt;          \
    uint16_t i2c0_over_under_cnt        = i2c0.errors->over_under_cnt;        \
    uint16_t i2c0_pec_recep_cnt         = i2c0.errors->pec_recep_cnt;         \
    uint16_t i2c0_timeout_tlow_cnt      = i2c0.errors->timeout_tlow_cnt;      \
    uint16_t i2c0_smbus_alert_cnt       = i2c0.errors->smbus_alert_cnt;       \
    uint16_t i2c0_unexpected_event_cnt  = i2c0.errors->unexpected_event_cnt;  \
    uint32_t i2c0_last_unexpected_event = i2c0.errors->last_unexpected_event; \
    const uint8_t _bus0 = 0;                                            \
    DOWNLINK_SEND_I2C_ERRORS(_trans, _dev,                  \
                             &i2c0_ack_fail_cnt,            \
                             &i2c0_miss_start_stop_cnt,     \
                             &i2c0_arb_lost_cnt,            \
                             &i2c0_over_under_cnt,          \
                             &i2c0_pec_recep_cnt,           \
                             &i2c0_timeout_tlow_cnt,        \
                             &i2c0_smbus_alert_cnt,         \
                             &i2c0_unexpected_event_cnt,    \
                             &i2c0_last_unexpected_event,   \
                             &_bus0);                       \
  }
#else
#define PERIODIC_SEND_I2C0_ERRORS(_trans, _dev) {}
#endif

#ifdef USE_I2C1
#define PERIODIC_SEND_I2C1_ERRORS(_trans, _dev) {                             \
    uint16_t i2c1_ack_fail_cnt          = i2c1.errors->ack_fail_cnt;          \
    uint16_t i2c1_miss_start_stop_cnt   = i2c1.errors->miss_start_stop_cnt;   \
    uint16_t i2c1_arb_lost_cnt          = i2c1.errors->arb_lost_cnt;          \
    uint16_t i2c1_over_under_cnt        = i2c1.errors->over_under_cnt;        \
    uint16_t i2c1_pec_recep_cnt         = i2c1.errors->pec_recep_cnt;         \
    uint16_t i2c1_timeout_tlow_cnt      = i2c1.errors->timeout_tlow_cnt;      \
    uint16_t i2c1_smbus_alert_cnt       = i2c1.errors->smbus_alert_cnt;       \
    uint16_t i2c1_unexpected_event_cnt  = i2c1.errors->unexpected_event_cnt;  \
    uint32_t i2c1_last_unexpected_event = i2c1.errors->last_unexpected_event; \
    const uint8_t _bus1 = 1;                                            \
    DOWNLINK_SEND_I2C_ERRORS(_trans, _dev,                  \
                             &i2c1_ack_fail_cnt,            \
                             &i2c1_miss_start_stop_cnt,     \
                             &i2c1_arb_lost_cnt,            \
                             &i2c1_over_under_cnt,          \
                             &i2c1_pec_recep_cnt,           \
                             &i2c1_timeout_tlow_cnt,        \
                             &i2c1_smbus_alert_cnt,         \
                             &i2c1_unexpected_event_cnt,    \
                             &i2c1_last_unexpected_event,   \
                             &_bus1);                       \
  }
#else
#define PERIODIC_SEND_I2C1_ERRORS(_trans, _dev) {}
#endif

#ifdef USE_I2C2
#define PERIODIC_SEND_I2C2_ERRORS(_trans, _dev) {                             \
    uint16_t i2c2_ack_fail_cnt          = i2c2.errors->ack_fail_cnt;          \
    uint16_t i2c2_miss_start_stop_cnt   = i2c2.errors->miss_start_stop_cnt;   \
    uint16_t i2c2_arb_lost_cnt          = i2c2.errors->arb_lost_cnt;          \
    uint16_t i2c2_over_under_cnt        = i2c2.errors->over_under_cnt;        \
    uint16_t i2c2_pec_recep_cnt         = i2c2.errors->pec_recep_cnt;         \
    uint16_t i2c2_timeout_tlow_cnt      = i2c2.errors->timeout_tlow_cnt;      \
    uint16_t i2c2_smbus_alert_cnt       = i2c2.errors->smbus_alert_cnt;       \
    uint16_t i2c2_unexpected_event_cnt  = i2c2.errors->unexpected_event_cnt;  \
    uint32_t i2c2_last_unexpected_event = i2c2.errors->last_unexpected_event; \
    const uint8_t _bus2 = 2;                                            \
    DOWNLINK_SEND_I2C_ERRORS(_trans, _dev,                  \
                             &i2c2_ack_fail_cnt,            \
                             &i2c2_miss_start_stop_cnt,     \
                             &i2c2_arb_lost_cnt,            \
                             &i2c2_over_under_cnt,          \
                             &i2c2_pec_recep_cnt,           \
                             &i2c2_timeout_tlow_cnt,        \
                             &i2c2_smbus_alert_cnt,         \
                             &i2c2_unexpected_event_cnt,    \
                             &i2c2_last_unexpected_event,   \
                             &_bus2);                       \
  }
#else
#define PERIODIC_SEND_I2C2_ERRORS(_trans, _dev) {}
#endif

#ifndef SITL
#define PERIODIC_SEND_I2C_ERRORS(_trans, _dev) {        \
    static uint8_t _i2c_nb_cnt = 0;                     \
    switch (_i2c_nb_cnt) {                              \
      case 0:                                           \
        PERIODIC_SEND_I2C0_ERRORS(_trans, _dev); break; \
      case 1:                                           \
        PERIODIC_SEND_I2C1_ERRORS(_trans, _dev); break; \
      case 2:                                           \
        PERIODIC_SEND_I2C2_ERRORS(_trans, _dev); break; \
      default:                                          \
        break;                                          \
    }                                                   \
    _i2c_nb_cnt++;                                      \
    if (_i2c_nb_cnt == 3)                               \
      _i2c_nb_cnt = 0;                                  \
  }
#else
#define PERIODIC_SEND_I2C_ERRORS(_trans, _dev) {}
#endif

#ifdef BOOZ2_TRACK_CAM
#include "cam_track.h"
#define PERIODIC_SEND_CAM_TRACK(_trans, _dev) DOWNLINK_SEND_NPS_SPEED_POS(_trans, _dev, \
    &target_accel_ned.x, \
    &target_accel_ned.y, \
    &target_accel_ned.z, \
    &target_speed_ned.x, \
    &target_speed_ned.y, \
    &target_speed_ned.z, \
    &target_pos_ned.x, \
    &target_pos_ned.y, \
    &target_pos_ned.z)
#else
#define PERIODIC_SEND_CAM_TRACK(_trans, _dev) {}
#endif

#ifdef ARDRONE2_RAW
#include "navdata.h"
#define PERIODIC_SEND_ARDRONE_NAVDATA(_trans, _dev) DOWNLINK_SEND_ARDRONE_NAVDATA(_trans, _dev, \
	&navdata->taille, \
	&navdata->nu_trame, \
	&navdata->ax, \
	&navdata->ay, \
	&navdata->az, \
	&navdata->vx, \
	&navdata->vy, \
	&navdata->vz, \
	&navdata->temperature_acc, \
	&navdata->temperature_gyro, \
	&navdata->ultrasound, \
	&navdata->us_debut_echo, \
	&navdata->us_fin_echo, \
	&navdata->us_association_echo, \
	&navdata->us_distance_echo, \
	&navdata->us_curve_time, \
	&navdata->us_curve_value, \
	&navdata->us_curve_ref, \
	&navdata->nb_echo, \
	&navdata->sum_echo, \
	&navdata->gradient, \
	&navdata->flag_echo_ini, \
	&navdata->pressure, \
	&navdata->temperature_pressure, \
	&navdata->mx, \
	&navdata->my, \
	&navdata->mz, \
	&navdata->chksum \
	)
#else
#define PERIODIC_SEND_ARDRONE_NAVDATA(_trans, _dev) {}
#endif

#include "generated/settings.h"
#define PERIODIC_SEND_DL_VALUE(_trans, _dev) PeriodicSendDlValue(_trans, _dev)

/*
 * Sending of UART errors.
 */
#ifdef USE_UART1
#define PERIODIC_SEND_UART1_ERRORS(_trans, _dev) {   \
    const uint8_t _bus1 = 1;                         \
    DOWNLINK_SEND_UART_ERRORS(_trans, _dev,          \
                             &uart1.ore,             \
                             &uart1.ne_err,          \
                             &uart1.fe_err,          \
                             &_bus1);                \
  }
#else
#define PERIODIC_SEND_UART1_ERRORS(_trans, _dev) {}
#endif

#ifdef USE_UART2
#define PERIODIC_SEND_UART2_ERRORS(_trans, _dev) {   \
    const uint8_t _bus2 = 2;                         \
    DOWNLINK_SEND_UART_ERRORS(_trans, _dev,          \
                             &uart2.ore,             \
                             &uart2.ne_err,          \
                             &uart2.fe_err,          \
                             &_bus2);                \
  }
#else
#define PERIODIC_SEND_UART2_ERRORS(_trans, _dev) {}
#endif

#ifdef USE_UART3
#define PERIODIC_SEND_UART3_ERRORS(_trans, _dev) {   \
    const uint8_t _bus3 = 3;                         \
    DOWNLINK_SEND_UART_ERRORS(_trans, _dev,          \
                             &uart3.ore,             \
                             &uart3.ne_err,          \
                             &uart3.fe_err,          \
                             &_bus3);                \
  }
#else
#define PERIODIC_SEND_UART3_ERRORS(_trans, _dev) {}
#endif

#ifdef USE_UART5
#define PERIODIC_SEND_UART5_ERRORS(_trans, _dev) {   \
    const uint8_t _bus5 = 5;                         \
    DOWNLINK_SEND_UART_ERRORS(_trans, _dev,          \
                             &uart5.ore,             \
                             &uart5.ne_err,          \
                             &uart5.fe_err,          \
                             &_bus5);                \
  }
#else
#define PERIODIC_SEND_UART5_ERRORS(_trans, _dev) {}
#endif


#ifndef SITL
#define PERIODIC_SEND_UART_ERRORS(_trans, _dev) {           \
    static uint8_t uart_nb_cnt = 0;                         \
    switch (uart_nb_cnt) {                                  \
      case 0:                                               \
        PERIODIC_SEND_UART1_ERRORS(_trans, _dev); break;    \
      case 1:                                               \
        PERIODIC_SEND_UART2_ERRORS(_trans, _dev); break;    \
      case 2:                                               \
        PERIODIC_SEND_UART3_ERRORS(_trans, _dev); break;    \
      case 3:                                               \
        PERIODIC_SEND_UART5_ERRORS(_trans, _dev); break;    \
      default: break;                                       \
    }                                                       \
    uart_nb_cnt++;                                          \
    if (uart_nb_cnt == 4)                                   \
      uart_nb_cnt = 0;                                      \
  }
#else
#define PERIODIC_SEND_UART_ERRORS(_trans, _dev) {}
#endif

#endif /* TELEMETRY_H */
