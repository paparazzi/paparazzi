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
#include "subsystems/abi.h"
#include "subsystems/imu.h"
#include "state.h"

/* Main magneto structure */
static struct mag_pitot_t mag_pitot = {
  .device = (&((MAG_PITOT_PORT).device)),
  .msg_available = false
};
static uint8_t mp_msg_buf[128]  __attribute__((aligned));   ///< The message buffer for the Magneto and pitot


#ifndef MAG_PITOT_REMOTE_GROUND_AMOUNT_SENSORS
#define MAG_PITOT_REMOTE_GROUND_AMOUNT_SENSORS 0
#endif

#ifndef MAG_PITOT_REMOTE_GROUND_ORIENTATIONS
#define MAG_PITOT_REMOTE_GROUND_ORIENTATIONS 0,0,0
#endif

static uint16_t remote_ground_value_array[MAG_PITOT_REMOTE_GROUND_AMOUNT_SENSORS];
static int32_t remote_ground_orientation_array[MAG_PITOT_REMOTE_GROUND_AMOUNT_SENSORS * 3];


#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void mag_pitot_raw_downlink(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_IMU_MAG_RAW(trans, dev, AC_ID, &imu.mag_unscaled.x, &imu.mag_unscaled.y,
                            &imu.mag_unscaled.z);
}
#endif

/* Initialize the magneto and pitot */
void mag_pitot_init()
{
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
  uint8_t msg_id = mp_msg_buf[1];
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

      AbiSendMsgBARO_ABS(IMU_MAG_PITOT_ID, pitot_stat);
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

    /* Get Time of Flight laser range sensor ring  messages */
    case DL_IMCU_REMOTE_GROUND: {
      // Retrieve ID and Range of the laser range sensor
      uint8_t id = DL_IMCU_REMOTE_GROUND_id(mp_msg_buf);
      uint16_t range = DL_IMCU_REMOTE_GROUND_range(mp_msg_buf);

      /* If the retrieved ID is the same or smaller that the total amount of specified sensors,continue
       */
      if (id <= MAG_PITOT_REMOTE_GROUND_AMOUNT_SENSORS - 1) {
        //Save the range and the orientation in the specified index (as defined in the airframe file
        remote_ground_value_array[id] = range;
        int16_t length = MAG_PITOT_REMOTE_GROUND_AMOUNT_SENSORS;
        static float remote_ground_orientation_array_float[] = {MAG_PITOT_REMOTE_GROUND_ORIENTATIONS};
        for (int n = 0; n < 3; n++) {
          remote_ground_orientation_array[id * 3 + n] =
            ANGLE_BFP_OF_REAL(remote_ground_orientation_array_float[id * 3 + n]);
        }
        //Send the abi message to be used by
        AbiSendMsgRANGE_SENSORS_ARRAY(RANGE_SENSOR_ARRAY_VL53L0_ID, length, remote_ground_value_array, remote_ground_orientation_array);

        //If an AGL_sonar orientation is defined, send this also by ABI
#ifdef MAG_PITOT_REMOTE_GROUND_ORIENTATION_AGL

        static float remote_ground_orientation_agl_array_float[] = {MAG_PITOT_REMOTE_GROUND_ORIENTATION_AGL};
        int32_t check_phi = ANGLE_BFP_OF_REAL(remote_ground_orientation_agl_array_float[0]);
        int32_t check_theta = ANGLE_BFP_OF_REAL(remote_ground_orientation_agl_array_float[1]);
        int32_t check_psi = ANGLE_BFP_OF_REAL(remote_ground_orientation_agl_array_float[2]);

        if (RadOfDeg(5) > fabs(remote_ground_orientation_array[id * 3] - check_psi)
            && RadOfDeg(5) > fabs(remote_ground_orientation_array[id * 3 + 1] - check_theta)
            && RadOfDeg(5) > fabs(remote_ground_orientation_array[id * 3 + 2] - check_psi)) {
          float agl = (float)range / 1000.;
          AbiSendMsgAGL(AGL_VL53L0_LASER_ARRAY_ID, agl);
        }
#endif
      }

      break;
    }

    default:
      break;
  }
}

/* We need to wait for incomming messages */
void mag_pitot_event()
{
  // Check if we got some message from the Magneto or Pitot
  pprz_check_and_parse(mag_pitot.device, &mag_pitot.transport, mp_msg_buf, &mag_pitot.msg_available);

  // If we have a message we should parse it
  if (mag_pitot.msg_available) {
    mag_pitot_parse_msg();
    mag_pitot.msg_available = false;
  }
}


