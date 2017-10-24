/*
 * Copyright (C)
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
 * @file "modules/laser_range_array/laser_range_array.c"
 * @author K. N. McGuire
 * Reads out values through uart of an laser range ring (array), containing multiple ToF IR laser range modules
 */

#include "modules/range_finder/laser_range_array.h"

#include "generated/airframe.h"
#include "pprzlink/pprz_transport.h"
#include "pprzlink/intermcu_msg.h"
#include "mcu_periph/uart.h"
#include "subsystems/abi.h"
#include "subsystems/imu.h"
#include "state.h"

/* Main magneto structure */
static struct laser_range_array_t laser_range_array = {
  .device = (&((LASER_RANGE_ARRAY_PORT).device)),
  .msg_available = false
};
static uint8_t lra_msg_buf[128]  __attribute__((aligned));   ///< The message buffer for the Magneto and pitot

#ifndef LASER_RANGE_ARRAY_NUMBER_SENSORS
#define LASER_RANGE_ARRAY_NUMBER_SENSORS 0
#endif
PRINT_CONFIG_VAR(LASER_RANGE_ARRAY_AMOUNT_SENSORS)


#ifndef LASER_RANGE_ARRAY_ORIENTATIONS
#define LASER_RANGE_ARRAY_ORIENTATIONS 0. ,0., 0.
#endif


uint16_t laser_range_array_values[LASER_RANGE_ARRAY_NUMBER_SENSORS];
double laser_range_array_orientations_d[] = {LASER_RANGE_ARRAY_ORIENTATIONS};
#ifdef LASER_RANGE_ARRAY_ORIENTATION_AGL
float laser_range_array_orientation_agl_d[] = {LASER_RANGE_ARRAY_ORIENTATION_AGL};
#endif

void laser_ring_array_init(void)
{
  // Initialize transport protocol
  pprz_transport_init(&laser_range_array.transport);
}

/* Parse the InterMCU message */
static inline void laser_range_array_parse_msg(void)
{
  /* Parse the message */
  uint8_t msg_id = lra_msg_buf[1];
  /* Get Time of Flight laser range sensor ring  messages */
  switch (msg_id) {

    case DL_IMCU_REMOTE_GROUND: {
      // Retrieve ID and Range of the laser range sensor
      uint8_t id = DL_IMCU_REMOTE_GROUND_id(lra_msg_buf);
      uint16_t range = DL_IMCU_REMOTE_GROUND_range(lra_msg_buf);

      /* If the retrieved ID is the same or smaller that the total amount of specified sensors,continue
       */
      if (id <= LASER_RANGE_ARRAY_NUMBER_SENSORS - 1) {
        //Save the range and the orientation in the specified index (as defined in the airframe file
        laser_range_array_values[id] = range;
        int16_t length = LASER_RANGE_ARRAY_NUMBER_SENSORS;


        //Send the abi message to be used by
        AbiSendMsgRANGE_SENSORS_ARRAY(RANGE_SENSOR_ARRAY_VL53L0_ID, length, laser_range_array_values, laser_range_array_orientations_d);

        //If an AGL_sonar orientation is defined, send this also by ABI
#ifdef LASER_RANGE_ARRAY_ORIENTATION_AGL
        float check_phi = laser_range_array_orientation_agl_d[0];
        float check_theta = laser_range_array_orientation_agl_d[1];
        float check_psi = laser_range_array_orientation_agl_d[2];

        if (RadOfDeg(5) > fabs(laser_range_array_orientations_d[id * 3] - check_phi)
            && RadOfDeg(5) > fabs(laser_range_array_orientations_d[id * 3 + 1] - check_theta)
            && RadOfDeg(5) > fabs(laser_range_array_orientations_d[id * 3 + 2] - check_psi)) {
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
void laser_range_array_event()
{
  // Check if we got some message from the Magneto or Pitot
  pprz_check_and_parse(laser_range_array.device, &laser_range_array.transport, lra_msg_buf, &laser_range_array.msg_available);

  // If we have a message we should parse it
  if (laser_range_array.msg_available) {
    laser_range_array_parse_msg();
    laser_range_array.msg_available = false;
  }

}


