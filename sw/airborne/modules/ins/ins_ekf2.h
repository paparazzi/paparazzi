/*
 * Copyright (C) 2016 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file modules/ins/ins_ekf2.h
 *
 * INS based in the EKF2 of PX4
 *
 */

#ifndef INS_EKF2_H
#define INS_EKF2_H

#ifdef __cplusplus
extern "C" {
#endif

#include "modules/ahrs/ahrs.h"
#include "modules/ins/ins.h"

/* Main EKF2 structure for keeping track of the status and use cross messaging */
struct ekf2_t {
  struct FloatRates delta_gyro;   ///< Last gyroscope measurements
  struct FloatVect3 delta_accel;  ///< Last accelerometer measurements
  uint32_t gyro_dt;               ///< Gyroscope delta timestamp between abi messages (us)
  uint32_t accel_dt;              ///< Accelerometer delta timestamp between abi messages (us)
  bool gyro_valid;                ///< If we received a gyroscope measurement
  bool accel_valid;               ///< If we received a acceleration measurement
  uint32_t flow_stamp;            ///< Optic flow last abi message timestamp

  float temp;                     ///< Latest temperature measurement in degrees celcius
  float qnh;                      ///< QNH value in hPa
  uint8_t quat_reset_counter;     ///< Amount of quaternion resets from the EKF2
  uint64_t ltp_stamp;             ///< Last LTP change timestamp from the EKF2
  struct LtpDef_i ltp_def;        ///< Latest LTP definition from the quat_reset_counter EKF2
  bool got_imu_data;              ///< If we received valid IMU data (any sensor)

  int32_t mag_fusion_type;
  int32_t fusion_mode;
};

extern void ins_ekf2_init(void);
extern void ins_ekf2_update(void);
extern void ins_ekf2_change_param(int32_t unk);
extern void ins_ekf2_remove_gps(int32_t mode);
extern void ins_ekf2_parse_EXTERNAL_POSE(uint8_t *buf);
extern void ins_ekf2_parse_EXTERNAL_POSE_SMALL(uint8_t *buf);
extern struct ekf2_t ekf2;

#ifdef __cplusplus
}
#endif

#endif /* INS_EKF2_H */
