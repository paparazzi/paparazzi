/*
 * Copyright (C) 2013 Michal Podhradsky
 * Utah State University, http://aggieair.usu.edu/
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
 * @file subsystems/ahrs/ahrs_extern_quat.c
 *
 * AHRS interface for multiple IMU/AHRS subsystems, such as GX3, UM6 etc.
 *
 * Propagates the estimated attitude and rates from IMU to body states. Quaternion
 * calculation is used.
 *
 * @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 */
#include "ahrs_extern_quat.h"
#include "mcu_periph/sys_time.h"
#include "led.h"

struct AhrsIntExternQuat ahrs_impl;

void ahrs_init(void) {
  ahrs.status = AHRS_UNINIT;

  /* set ltp_to_imu so that body is zero */
  QUAT_COPY(ahrs_impl.ltp_to_imu_quat, imu.body_to_imu_quat);
  INT_RATES_ZERO(ahrs_impl.imu_rate);

  #ifdef IMU_MAG_OFFSET
    ahrs_impl.mag_offset = IMU_MAG_OFFSET;
  #else
    ahrs_impl.mag_offset = 0.;
  #endif

  //Needed to set orientations
  ahrs.status = AHRS_RUNNING;

  #ifdef AHRS_ALIGNER_LED
      LED_ON(AHRS_ALIGNER_LED);
  #endif
}

void ahrs_propagate(void) {
  /* Compute LTP to BODY quaternion */
  struct Int32Quat ltp_to_body_quat;
  INT32_QUAT_COMP_INV(ltp_to_body_quat, ahrs_impl.ltp_to_imu_quat, imu.body_to_imu_quat);
  stateSetNedToBodyQuat_i(&ltp_to_body_quat);

  // TODO: compensate for magnetic offset

  struct Int32Rates body_rate;
  ahrs_impl.imu_rate = imu.gyro;
  /* compute body rates */
  INT32_RMAT_TRANSP_RATEMULT(body_rate, imu.body_to_imu_rmat, ahrs_impl.imu_rate);
  /* Set state */
  stateSetBodyRates_i(&body_rate);
}

void ahrs_align(void) {
}

void ahrs_update_accel(void) {
}

void ahrs_update_mag(void) {
}

void ahrs_update_gps(void) {
}
