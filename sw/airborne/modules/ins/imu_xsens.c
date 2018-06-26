/*
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
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
 *
 */

/** @file imu_xsens.c
 * XSENS to just provide IMU measurements.
 * For use with an external AHRS algorithm.
 */

#include "imu_xsens.h"
#include "xsens.h"

#include "generated/airframe.h"

#include "mcu_periph/sys_time.h"
#include "subsystems/abi.h"

static void handle_ins_msg(void);

void imu_xsens_init(void)
{
  xsens_init();
}

void imu_xsens_event(void)
{
  xsens_parser_event(&(xsens.parser));
  if (xsens.parser.msg_received) {
    parse_xsens_msg();
    handle_ins_msg();
    xsens.parser.msg_received = FALSE;
  }
}

static void handle_ins_msg(void)
{
  uint32_t now_ts = get_sys_time_usec();
#ifdef XSENS_BACKWARDS
  if (xsens.gyro_available) {
    RATES_ASSIGN(imu.gyro_unscaled, -RATE_BFP_OF_REAL(xsens.gyro.p), -RATE_BFP_OF_REAL(xsens.gyro.q), RATE_BFP_OF_REAL(xsens.gyro.r));
     xsens.gyro_available = FALSE;
     imu_scale_gyro(&imu);
     AbiSendMsgIMU_GYRO_INT32(IMU_XSENS_ID, now_ts, &imu.gyro);
  }
  if (xsens.accel_available) {
    VECT3_ASSIGN(imu.accel_unscaled, -ACCEL_BFP_OF_REAL(xsens.accel.ax), -ACCEL_BFP_OF_REAL(xsens.accel.ay), ACCEL_BFP_OF_REAL(xsens.accel.az));
    xsens.accel_available = FALSE;
    imu_scale_accel(&imu);
    AbiSendMsgIMU_ACCEL_INT32(IMU_XSENS_ID, now_ts, &imu.accel);
  }
  if (xsens.mag_available) {
    VECT3_ASSIGN(imu.mag_unscaled, -MAG_BFP_OF_REAL(xsens.mag.mx), -MAG_BFP_OF_REAL(xsens.mag.my), MAG_BFP_OF_REAL(xsens.mag.mz));
    xsens.mag_available = FALSE;
    imu_scale_mag(&imu);
    AbiSendMsgIMU_MAG_INT32(IMU_XSENS_ID, now_ts, &imu.mag);
  }
#else
  if (xsens.gyro_available) {
    RATES_BFP_OF_REAL(imu.gyro_unscaled, xsens.gyro);
    xsens.gyro_available = FALSE;
    imu_scale_gyro(&imu);
    AbiSendMsgIMU_GYRO_INT32(IMU_XSENS_ID, now_ts, &imu.gyro);
  }
  if (xsens.accel_available) {
    ACCELS_BFP_OF_REAL(imu.accel_unscaled, xsens.accel);
    xsens.accel_available = FALSE;
    imu_scale_accel(&imu);
    AbiSendMsgIMU_ACCEL_INT32(IMU_XSENS_ID, now_ts, &imu.accel);
  }
  if (xsens.mag_available) {
    MAGS_BFP_OF_REAL(imu.mag_unscaled, xsens.mag);
    xsens.mag_available = FALSE;
    imu_scale_mag(&imu);
    AbiSendMsgIMU_MAG_INT32(IMU_XSENS_ID, now_ts, &imu.mag);
  }
#endif /* XSENS_BACKWARDS */
}
