/*
 * Copyright (C) 2016 Michal Podhradsky, michal.podhradsky@aggiemail.usu.edu
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

/**
 * @file modules/imu/imu_vectornav.c
 *
 * Vectornav VN-200 IMU module, to be used with other AHRS/INS algorithms.
 */

#include "modules/imu/imu_vectornav.h"

// Systime
#include "mcu_periph/sys_time.h"

// Abi
#include "modules/core/abi.h"

// Generated
#include "generated/airframe.h"

struct ImuVectornav imu_vn;

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"


static void send_vn_info(struct transport_tx *trans, struct link_device *dev)
{
  static uint16_t last_cnt = 0;
  static uint16_t sec_cnt = 0;

  sec_cnt = imu_vn.vn_packet.counter -  last_cnt;
  imu_vn.vn_freq = sec_cnt; // update frequency counter

  pprz_msg_send_VECTORNAV_INFO(trans, dev, AC_ID,
                               &imu_vn.vn_data.timestamp,
                               &imu_vn.vn_packet.chksm_error,
                               &imu_vn.vn_packet.hdr_error,
                               &sec_cnt,
                               &imu_vn.vn_data.mode,
                               &imu_vn.vn_data.err,
                               &imu_vn.vn_data.ypr_u.phi,
                               &imu_vn.vn_data.ypr_u.theta,
                               &imu_vn.vn_data.ypr_u.psi);
  // update counter
  last_cnt = imu_vn.vn_packet.counter;

  // reset mode
  imu_vn.vn_data.mode = 0;
}
#endif


/**
 * Init IMU struct and set up ABI messages
 */
void imu_vectornav_init(void)
{
  // Initialize variables
  imu_vn.vn_status = VNNotTracking;
  imu_vn.vn_freq = 0;

  // Initialize packet
  imu_vn.vn_packet.status = VNMsgSync;
  imu_vn.vn_packet.msg_idx = 0;
  imu_vn.vn_packet.msg_available = false;
  imu_vn.vn_packet.chksm_error = 0;
  imu_vn.vn_packet.hdr_error = 0;
  imu_vn.vn_packet.overrun_error = 0;
  imu_vn.vn_packet.noise_error = 0;
  imu_vn.vn_packet.framing_error = 0;

  // initialize data structs
  memset(&(imu_vn.vn_data), 0, sizeof(struct VNData));

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_VECTORNAV_INFO, send_vn_info);
#endif
}



/**
 * Event function to read and parse data from the serial port
 */
void imu_vectornav_event(void)
{
  // receive data
  vn200_event(&(imu_vn.vn_packet));

  // read message
  if (imu_vn.vn_packet.msg_available) {
    vn200_read_message(&(imu_vn.vn_packet),&(imu_vn.vn_data));
    imu_vn.vn_packet.msg_available = false;
    imu_vectornav_propagate();
  }
}


/**
 * Periodic function checks for the frequency of packets,
 * triggers warning in case the IMU stops sending data
 * and performs initial configuration if needed
 */
void imu_vectornav_periodic(void)
{
  static uint16_t last_cnt = 0;
  static uint16_t sec_cnt = 0;

  sec_cnt = imu_vn.vn_packet.counter -  last_cnt;
  imu_vn.vn_freq = sec_cnt; // update frequency counter

  // we want at least 75% of periodic frequency to be able to control the airfcraft
  if (imu_vn.vn_freq >= (PERIODIC_FREQUENCY*0.75)) {
    imu_vn.vn_status = VNNotTracking;
  }
  else {
    imu_vn.vn_status = VNOK;
  }
}


/**
 * Send ABI messages
 */
void imu_vectornav_propagate(void)
{
  uint64_t tmp = imu_vn.vn_data.nanostamp / 1000;
  uint32_t now_ts = (uint32_t) tmp;

  // Send accel and gyro data separate for other AHRS algorithms
  AbiSendMsgIMU_GYRO_RAW(IMU_VECTORNAV_ID, now_ts, &imu_vn.vn_data.gyro, 1);
  AbiSendMsgIMU_ACCEL_RAW(IMU_VECTORNAV_ID, now_ts, &imu_vn.vn_data.accel, 1);
}

