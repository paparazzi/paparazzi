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
#include "modules/core/abi.h"

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
    struct Int32Rates gyro = {
      -RATE_BFP_OF_REAL(xsens.gyro.p),
      -RATE_BFP_OF_REAL(xsens.gyro.q),
      RATE_BFP_OF_REAL(xsens.gyro.r)
    };
    xsens.gyro_available = FALSE
    AbiSendMsgIMU_GYRO_RAW(IMU_XSENS_ID, now_ts, &gyro, 1, NAN);
  }
  if (xsens.accel_available) {
    struct Int32Vect3 accel = {
      -ACCEL_BFP_OF_REAL(xsens.accel.x),
      -ACCEL_BFP_OF_REAL(xsens.accel.y),
      ACCEL_BFP_OF_REAL(xsens.accel.z)
    };
    xsens.accel_available = FALSE;
    AbiSendMsgIMU_ACCEL_RAW(IMU_XSENS_ID, now_ts, &accel, 1, NAN);
  }
  if (xsens.mag_available) {
    struct Int32Vect3 mag = {
      -MAG_BFP_OF_REAL(xsens.mag.x),
      -MAG_BFP_OF_REAL(xsens.mag.y),
      MAG_BFP_OF_REAL(xsens.mag.z)
    };
    xsens.mag_available = FALSE;
    AbiSendMsgIMU_MAG_RAW(IMU_XSENS_ID, now_ts, &mag);
  }
#else
  if (xsens.gyro_available) {
    struct Int32Rates gyro = {
      RATE_BFP_OF_REAL(xsens.gyro.p),
      RATE_BFP_OF_REAL(xsens.gyro.q),
      RATE_BFP_OF_REAL(xsens.gyro.r)
    };
    AbiSendMsgIMU_GYRO_RAW(IMU_XSENS_ID, now_ts, &gyro, 1, NAN);
    xsens.gyro_available = FALSE;
  }
  if (xsens.accel_available) {
    struct Int32Vect3 accel = {
      ACCEL_BFP_OF_REAL(xsens.accel.x),
      ACCEL_BFP_OF_REAL(xsens.accel.y),
      ACCEL_BFP_OF_REAL(xsens.accel.z)
    };
    AbiSendMsgIMU_ACCEL_RAW(IMU_XSENS_ID, now_ts, &accel, 1, NAN);
    xsens.accel_available = FALSE;
  }
  if (xsens.mag_available) {
    struct Int32Vect3 mag = {
      MAG_BFP_OF_REAL(xsens.mag.x),
      MAG_BFP_OF_REAL(xsens.mag.y),
      MAG_BFP_OF_REAL(xsens.mag.z)
    };
    AbiSendMsgIMU_MAG_RAW(IMU_XSENS_ID, now_ts, &mag);
    xsens.mag_available = FALSE;
  }
#endif /* XSENS_BACKWARDS */
}
