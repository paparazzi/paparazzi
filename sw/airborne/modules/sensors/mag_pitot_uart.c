/*
 * Copyright (C) C. De Wagter
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/sensors/mag_pitot_uart.c"
 * @author C. De Wagter
 * Remotely located magnetometer and pitot tube over uart (RS232) communication
 */

#include "modules/sensors/mag_pitot_uart.h"

#include "pprzlink/pprz_transport.h"
#include "pprzlink/intermcu_msg.h"
#include "mcu_periph/uart.h"
#include "modules/core/abi.h"
#include "modules/imu/imu.h"
#include "state.h"

/* Main magneto structure */
static struct mag_pitot_t mag_pitot = {
  .device = (&((MAG_PITOT_PORT).device)),
  .msg_available = false
};
static uint8_t mp_msg_buf[128]  __attribute__((aligned));   ///< The message buffer for the Magneto and pitot


#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void mag_pitot_raw_downlink(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_IMU_MAG_RAW(trans, dev, AC_ID, &imu.mag_unscaled.x, &imu.mag_unscaled.y,
                             &imu.mag_unscaled.z);
}
#endif

/* Initialize the magneto and pitot */
void mag_pitot_init() {
  // Initialize transport protocol
  pprz_transport_init(&mag_pitot.transport);

  // Set the Imu to Magneto rotation
  struct FloatEulers imu_to_mag_eulers =
    {IMU_TO_MAG_PHI, IMU_TO_MAG_THETA, IMU_TO_MAG_PSI};
  orientationSetEulers_f(&mag_pitot.imu_to_mag, &imu_to_mag_eulers);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_IMU_MAG_RAW, mag_pitot_raw_downlink);
#endif
}

/* Parse the InterMCU message */
static inline void mag_pitot_parse_msg(void)
{
  uint32_t now_ts = get_sys_time_usec();

  /* Parse the mag-pitot message */
  uint8_t msg_id = pprzlink_get_msg_id(mp_msg_buf);

#if PPRZLINK_DEFAULT_VER == 2
  // Check that the message is really a intermcu message
  if (pprzlink_get_msg_class_id(mp_msg_buf) == DL_intermcu_CLASS_ID) {
#endif
  switch (msg_id) {
  /* Got a magneto message */
  case DL_IMCU_REMOTE_MAG: {
    struct Int32Vect3 raw_mag;

    // Read the raw magneto information
    raw_mag.x = DL_IMCU_REMOTE_MAG_mag_x(mp_msg_buf);
    raw_mag.y = DL_IMCU_REMOTE_MAG_mag_y(mp_msg_buf);
    raw_mag.z = DL_IMCU_REMOTE_MAG_mag_z(mp_msg_buf);

    // Rotate the magneto
    struct Int32RMat *imu_to_mag_rmat = orientationGetRMat_i(&mag_pitot.imu_to_mag);
    int32_rmat_vmult(&imu.mag_unscaled, imu_to_mag_rmat, &raw_mag);

    // Send and set the magneto IMU data
    imu_scale_mag(&imu);
    AbiSendMsgIMU_MAG_INT32(IMU_MAG_PITOT_ID, now_ts, &imu.mag);
    break;
  }

  /* Got a barometer message */
  case DL_IMCU_REMOTE_BARO: {
    float pitot_stat = DL_IMCU_REMOTE_BARO_pitot_stat(mp_msg_buf);
    float pitot_temp = DL_IMCU_REMOTE_BARO_pitot_temp(mp_msg_buf);

    AbiSendMsgBARO_ABS(IMU_MAG_PITOT_ID, now_ts, pitot_stat);
    AbiSendMsgTEMPERATURE(IMU_MAG_PITOT_ID, pitot_temp);
    break;
  }

  /* Got an airspeed message */
  case DL_IMCU_REMOTE_AIRSPEED: {
    // Should be updated to differential pressure
    float pitot_ias = DL_IMCU_REMOTE_AIRSPEED_pitot_IAS(mp_msg_buf);
    AbiSendMsgAIRSPEED(IMU_MAG_PITOT_ID, pitot_ias);
    break;
  }

    default:
      break;
  }
#if PPRZLINK_DEFAULT_VER == 2
  }
#endif
}

/* We need to wait for incomming messages */
void mag_pitot_event() {
  // Check if we got some message from the Magneto or Pitot
  pprz_check_and_parse(mag_pitot.device, &mag_pitot.transport, mp_msg_buf, &mag_pitot.msg_available);

  // If we have a message we should parse it
  if (mag_pitot.msg_available) {
    mag_pitot_parse_msg();
    mag_pitot.msg_available = false;
  }
}


