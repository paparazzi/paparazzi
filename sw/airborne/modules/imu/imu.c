/*
 * Copyright (C) 2008-2010 The Paparazzi Team
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

/**
 * @file modules/imu/imu.c
 * Inertial Measurement Unit interface.
 */

#ifdef BOARD_CONFIG
#include BOARD_CONFIG
#endif

#include "modules/imu/imu.h"
#include "state.h"
#include "modules/core/abi.h"

#ifdef IMU_POWER_GPIO
#include "mcu_periph/gpio.h"

#ifndef IMU_POWER_GPIO_ON
#define IMU_POWER_GPIO_ON gpio_set
#endif
#endif

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_accel_raw(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_IMU_ACCEL_RAW(trans, dev, AC_ID,
                              &imu.accel_unscaled.x, &imu.accel_unscaled.y, &imu.accel_unscaled.z);
}

static void send_accel_scaled(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_IMU_ACCEL_SCALED(trans, dev, AC_ID,
                                 &imu.accel.x, &imu.accel.y, &imu.accel.z);
}

static void send_accel(struct transport_tx *trans, struct link_device *dev)
{
  struct FloatVect3 accel_float;
  ACCELS_FLOAT_OF_BFP(accel_float, imu.accel);
  pprz_msg_send_IMU_ACCEL(trans, dev, AC_ID,
                          &accel_float.x, &accel_float.y, &accel_float.z);
}

static void send_gyro_raw(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_IMU_GYRO_RAW(trans, dev, AC_ID,
                             &imu.gyro_unscaled.p, &imu.gyro_unscaled.q, &imu.gyro_unscaled.r);
}

static void send_gyro_scaled(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_IMU_GYRO_SCALED(trans, dev, AC_ID,
                                &imu.gyro.p, &imu.gyro.q, &imu.gyro.r);
}

static void send_gyro(struct transport_tx *trans, struct link_device *dev)
{
  struct FloatRates gyro_float;
  RATES_FLOAT_OF_BFP(gyro_float, imu.gyro);
  pprz_msg_send_IMU_GYRO(trans, dev, AC_ID,
                         &gyro_float.p, &gyro_float.q, &gyro_float.r);
}

static void send_mag_raw(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_IMU_MAG_RAW(trans, dev, AC_ID,
                            &imu.mag_unscaled.x, &imu.mag_unscaled.y, &imu.mag_unscaled.z);
}

static void send_mag_scaled(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_IMU_MAG_SCALED(trans, dev, AC_ID,
                               &imu.mag.x, &imu.mag.y, &imu.mag.z);
}

static void send_mag(struct transport_tx *trans, struct link_device *dev)
{
  struct FloatVect3 mag_float;
  MAGS_FLOAT_OF_BFP(mag_float, imu.mag);
  pprz_msg_send_IMU_MAG(trans, dev, AC_ID,
                        &mag_float.x, &mag_float.y, &mag_float.z);
}

#endif /* PERIODIC_TELEMETRY */

struct Imu imu;

void imu_init(void)
{

#ifdef IMU_POWER_GPIO
  gpio_setup_output(IMU_POWER_GPIO);
  IMU_POWER_GPIO_ON(IMU_POWER_GPIO);
#endif

  /* initialises neutrals */
  RATES_ASSIGN(imu.gyro_neutral,  IMU_GYRO_P_NEUTRAL,  IMU_GYRO_Q_NEUTRAL,  IMU_GYRO_R_NEUTRAL);

  VECT3_ASSIGN(imu.accel_neutral, IMU_ACCEL_X_NEUTRAL, IMU_ACCEL_Y_NEUTRAL, IMU_ACCEL_Z_NEUTRAL);

#if defined IMU_MAG_X_NEUTRAL && defined IMU_MAG_Y_NEUTRAL && defined IMU_MAG_Z_NEUTRAL
  VECT3_ASSIGN(imu.mag_neutral,   IMU_MAG_X_NEUTRAL,   IMU_MAG_Y_NEUTRAL,   IMU_MAG_Z_NEUTRAL);
#else
#if USE_MAGNETOMETER && (!defined MAG_CALIB_UKF_H)
  INFO("Magnetometer neutrals are set to zero, you should calibrate!")
#endif
  INT_VECT3_ZERO(imu.mag_neutral);
#endif

  struct FloatEulers body_to_imu_eulers =
  {IMU_BODY_TO_IMU_PHI, IMU_BODY_TO_IMU_THETA, IMU_BODY_TO_IMU_PSI};
  orientationSetEulers_f(&imu.body_to_imu, &body_to_imu_eulers);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_IMU_ACCEL_RAW, send_accel_raw);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_IMU_ACCEL_SCALED, send_accel_scaled);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_IMU_ACCEL, send_accel);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_IMU_GYRO_RAW, send_gyro_raw);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_IMU_GYRO_SCALED, send_gyro_scaled);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_IMU_GYRO, send_gyro);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_IMU_MAG_RAW, send_mag_raw);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_IMU_MAG_SCALED, send_mag_scaled);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_IMU_MAG, send_mag);
#endif // DOWNLINK

}


void imu_SetBodyToImuPhi(float phi)
{
  struct FloatEulers body_to_imu_eulers;
  body_to_imu_eulers = *orientationGetEulers_f(&imu.body_to_imu);
  body_to_imu_eulers.phi = phi;
  orientationSetEulers_f(&imu.body_to_imu, &body_to_imu_eulers);
  AbiSendMsgBODY_TO_IMU_QUAT(1, orientationGetQuat_f(&imu.body_to_imu));
}

void imu_SetBodyToImuTheta(float theta)
{
  struct FloatEulers body_to_imu_eulers;
  body_to_imu_eulers = *orientationGetEulers_f(&imu.body_to_imu);
  body_to_imu_eulers.theta = theta;
  orientationSetEulers_f(&imu.body_to_imu, &body_to_imu_eulers);
  AbiSendMsgBODY_TO_IMU_QUAT(1, orientationGetQuat_f(&imu.body_to_imu));
}

void imu_SetBodyToImuPsi(float psi)
{
  struct FloatEulers body_to_imu_eulers;
  body_to_imu_eulers = *orientationGetEulers_f(&imu.body_to_imu);
  body_to_imu_eulers.psi = psi;
  orientationSetEulers_f(&imu.body_to_imu, &body_to_imu_eulers);
  AbiSendMsgBODY_TO_IMU_QUAT(1, orientationGetQuat_f(&imu.body_to_imu));
}

void imu_SetBodyToImuCurrent(float set)
{
  imu.b2i_set_current = set;

  if (imu.b2i_set_current) {
    // adjust imu_to_body roll and pitch by current NedToBody roll and pitch
    struct FloatEulers body_to_imu_eulers;
    body_to_imu_eulers = *orientationGetEulers_f(&imu.body_to_imu);
    if (stateIsAttitudeValid()) {
      // adjust imu_to_body roll and pitch by current NedToBody roll and pitch
      body_to_imu_eulers.phi += stateGetNedToBodyEulers_f()->phi;
      body_to_imu_eulers.theta += stateGetNedToBodyEulers_f()->theta;
      orientationSetEulers_f(&imu.body_to_imu, &body_to_imu_eulers);
      AbiSendMsgBODY_TO_IMU_QUAT(1, orientationGetQuat_f(&imu.body_to_imu));
    } else {
      // indicate that we couldn't set to current roll/pitch
      imu.b2i_set_current = false;
    }
  } else {
    // reset to BODY_TO_IMU as defined in airframe file
    struct FloatEulers body_to_imu_eulers =
    {IMU_BODY_TO_IMU_PHI, IMU_BODY_TO_IMU_THETA, IMU_BODY_TO_IMU_PSI};
    orientationSetEulers_f(&imu.body_to_imu, &body_to_imu_eulers);
    AbiSendMsgBODY_TO_IMU_QUAT(1, orientationGetQuat_f(&imu.body_to_imu));
  }
}


// weak functions, used if not explicitly provided by implementation

void WEAK imu_scale_gyro(struct Imu *_imu)
{
  RATES_COPY(_imu->gyro_prev, _imu->gyro);
  _imu->gyro.p = ((_imu->gyro_unscaled.p - _imu->gyro_neutral.p) * IMU_GYRO_P_SIGN *
                  IMU_GYRO_P_SENS_NUM) / IMU_GYRO_P_SENS_DEN;
  _imu->gyro.q = ((_imu->gyro_unscaled.q - _imu->gyro_neutral.q) * IMU_GYRO_Q_SIGN *
                  IMU_GYRO_Q_SENS_NUM) / IMU_GYRO_Q_SENS_DEN;
  _imu->gyro.r = ((_imu->gyro_unscaled.r - _imu->gyro_neutral.r) * IMU_GYRO_R_SIGN *
                  IMU_GYRO_R_SENS_NUM) / IMU_GYRO_R_SENS_DEN;
}

void WEAK imu_scale_accel(struct Imu *_imu)
{
  VECT3_COPY(_imu->accel_prev, _imu->accel);
  _imu->accel.x = ((_imu->accel_unscaled.x - _imu->accel_neutral.x) * IMU_ACCEL_X_SIGN *
                   IMU_ACCEL_X_SENS_NUM) / IMU_ACCEL_X_SENS_DEN;
  _imu->accel.y = ((_imu->accel_unscaled.y - _imu->accel_neutral.y) * IMU_ACCEL_Y_SIGN *
                   IMU_ACCEL_Y_SENS_NUM) / IMU_ACCEL_Y_SENS_DEN;
  _imu->accel.z = ((_imu->accel_unscaled.z - _imu->accel_neutral.z) * IMU_ACCEL_Z_SIGN *
                   IMU_ACCEL_Z_SENS_NUM) / IMU_ACCEL_Z_SENS_DEN;
}

#if !defined SITL && defined IMU_MAG_X_CURRENT_COEF && defined IMU_MAG_Y_CURRENT_COEF && defined IMU_MAG_Z_CURRENT_COEF
#include "modules/energy/electrical.h"
void WEAK imu_scale_mag(struct Imu *_imu)
{
  struct Int32Vect3 mag_correction;
  mag_correction.x = (int32_t)(IMU_MAG_X_CURRENT_COEF * (float) electrical.current);
  mag_correction.y = (int32_t)(IMU_MAG_Y_CURRENT_COEF * (float) electrical.current);
  mag_correction.z = (int32_t)(IMU_MAG_Z_CURRENT_COEF * (float) electrical.current);
  _imu->mag.x = (((_imu->mag_unscaled.x - mag_correction.x) - _imu->mag_neutral.x) *
                 IMU_MAG_X_SIGN * IMU_MAG_X_SENS_NUM) / IMU_MAG_X_SENS_DEN;
  _imu->mag.y = (((_imu->mag_unscaled.y - mag_correction.y) - _imu->mag_neutral.y) *
                 IMU_MAG_Y_SIGN * IMU_MAG_Y_SENS_NUM) / IMU_MAG_Y_SENS_DEN;
  _imu->mag.z = (((_imu->mag_unscaled.z - mag_correction.z) - _imu->mag_neutral.z) *
                 IMU_MAG_Z_SIGN * IMU_MAG_Z_SENS_NUM) / IMU_MAG_Z_SENS_DEN;
}
#elif USE_MAGNETOMETER
void WEAK imu_scale_mag(struct Imu *_imu)
{
  _imu->mag.x = ((_imu->mag_unscaled.x - _imu->mag_neutral.x) * IMU_MAG_X_SIGN *
                 IMU_MAG_X_SENS_NUM) / IMU_MAG_X_SENS_DEN;
  _imu->mag.y = ((_imu->mag_unscaled.y - _imu->mag_neutral.y) * IMU_MAG_Y_SIGN *
                 IMU_MAG_Y_SENS_NUM) / IMU_MAG_Y_SENS_DEN;
  _imu->mag.z = ((_imu->mag_unscaled.z - _imu->mag_neutral.z) * IMU_MAG_Z_SIGN *
                 IMU_MAG_Z_SENS_NUM) / IMU_MAG_Z_SENS_DEN;
}
#else
void WEAK imu_scale_mag(struct Imu *_imu __attribute__((unused))) {}
#endif /* MAG_x_CURRENT_COEF */
