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
 */
/**
 * @file ahrs_vectornav.h
 *
 * Vectornav VN-200 as AHRS
 *
 * @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 */
#include "modules/ahrs/ahrs_vectornav.h"
#include "modules/ahrs/ahrs_vectornav_wrapper.h"
#include "modules/core/abi.h"
#include "state.h"

struct AhrsVectornav ahrs_vn;

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"


static void send_vn_info(struct transport_tx *trans, struct link_device *dev)
{
  static uint16_t last_cnt = 0;
  static uint16_t sec_cnt = 0;

  sec_cnt = ahrs_vn.vn_packet.counter -  last_cnt;
  ahrs_vn.vn_freq = sec_cnt; // update frequency counter

  pprz_msg_send_VECTORNAV_INFO(trans, dev, AC_ID,
                               &ahrs_vn.vn_data.timestamp,
                               &ahrs_vn.vn_packet.chksm_error,
                               &ahrs_vn.vn_packet.hdr_error,
                               &sec_cnt,
                               &ahrs_vn.vn_data.mode,
                               &ahrs_vn.vn_data.err,
                               &ahrs_vn.vn_data.ypr_u.phi,
                               &ahrs_vn.vn_data.ypr_u.theta,
                               &ahrs_vn.vn_data.ypr_u.psi);
  // update counter
  last_cnt = ahrs_vn.vn_packet.counter;

  // reset mode
  ahrs_vn.vn_data.mode = 0;
}
#endif

/**
 * Event handling for Vectornav
 */
void ahrs_vectornav_event(void)
{
  // receive data
  vn200_event(&(ahrs_vn.vn_packet));

  // read message
  if (ahrs_vn.vn_packet.msg_available) {
    vn200_read_message(&(ahrs_vn.vn_packet),&(ahrs_vn.vn_data));
    ahrs_vn.vn_packet.msg_available = false;

    if (ahrs_vectornav_is_enabled()) {
      ahrs_vectornav_propagate();
    }

    // send ABI messages
    uint32_t now_ts = get_sys_time_usec();
    // in fixed point for sending as ABI and telemetry msgs
    RATES_BFP_OF_REAL(ahrs_vn.gyro_i, ahrs_vn.vn_data.gyro);
    AbiSendMsgIMU_GYRO_RAW(IMU_VECTORNAV_ID, now_ts, &ahrs_vn.gyro_i, 1, NAN);
    ACCELS_BFP_OF_REAL(ahrs_vn.accel_i, ahrs_vn.vn_data.accel);
    AbiSendMsgIMU_ACCEL_RAW(IMU_VECTORNAV_ID, now_ts, &ahrs_vn.accel_i, 1, NAN);
  }
}


/**
 * Initialize Vectornav struct
 */
void ahrs_vectornav_init(void)
{
  // Initialize variables
  ahrs_vn.vn_status = VNNotTracking;
  ahrs_vn.vn_freq = 0;

  // Initialize packet
  ahrs_vn.vn_packet.status = VNMsgSync;
  ahrs_vn.vn_packet.msg_idx = 0;
  ahrs_vn.vn_packet.msg_available = false;
  ahrs_vn.vn_packet.chksm_error = 0;
  ahrs_vn.vn_packet.hdr_error = 0;
  ahrs_vn.vn_packet.overrun_error = 0;
  ahrs_vn.vn_packet.noise_error = 0;
  ahrs_vn.vn_packet.framing_error = 0;

  INT32_VECT3_ZERO(ahrs_vn.accel_i);

  // initialize data struct
  memset(&(ahrs_vn.vn_data), 0, sizeof(struct VNData));

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_VECTORNAV_INFO, send_vn_info);
#endif
}

/**
 * Read received data
 */
void ahrs_vectornav_propagate(void)
{
  // Rates [rad/s]
  static struct FloatRates body_rate;
  float_rmat_ratemult(&body_rate, orientationGetRMat_f(&ahrs_vn.body_to_imu), &ahrs_vn.vn_data.gyro); // compute body rates
  stateSetBodyRates_f(&body_rate);   // Set state [rad/s]

  // Attitude [deg]
  static struct FloatQuat imu_quat; // convert from euler to quat
  float_quat_of_eulers(&imu_quat, &ahrs_vn.vn_data.attitude);
  stateSetNedToBodyQuat_f(&imu_quat);

}


