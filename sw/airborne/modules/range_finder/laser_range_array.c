/*
 * Copyright (C) 2017 K. N. McGuire
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

/*
 * @file "modules/range_finder/laser_range_array.c"
 * @author K. N. McGuire
 * Reads out values through uart of an laser range ring (array), containing multiple ToF IR laser range modules
 */

#include "modules/range_finder/laser_range_array.h"

#include "generated/airframe.h"

#include "pprzlink/pprz_transport.h"
#include "pprzlink/intermcu_msg.h"
#include "mcu_periph/uart.h"

#include "subsystems/abi.h"

#include "message_pragmas.h"

/* Main device strcuture */
struct laser_range_array_t {
  struct link_device *device;           ///< The device which is uses for communication
  struct pprz_transport transport;      ///< The transport layer (PPRZ)
  bool msg_available;                   ///< If we received a message
};

static struct laser_range_array_t laser_range_array = {
  .device = (&((LASER_RANGE_ARRAY_PORT).device)),
  .msg_available = false
};

static uint8_t lra_msg_buf[128]  __attribute__((aligned));   ///< The message buffer

PRINT_CONFIG_VAR(LASER_RANGE_ARRAY_NUM_SENSORS)
uint16_t laser_range_array_values[LASER_RANGE_ARRAY_NUM_SENSORS];
float laser_range_array_orientations[] = LASER_RANGE_ARRAY_ORIENTATIONS;

uint8_t agl_id = 255;
uint8_t front_id = 255;

uint8_t id_recieved[LASER_RANGE_ARRAY_NUM_SENSORS] = {0};

void laser_range_array_init(void)
{
  // Initialize transport protocol
  pprz_transport_init(&laser_range_array.transport);

#if LASER_RANGE_ARRAY_SEND_AGL
  // Determine which sensor is looking down
  struct FloatEulers pose_down = {0., -M_PI_2, 0.};

  struct FloatQuat q_down;
  float_quat_of_eulers(&q_down, &pose_down);

  for (int k = 0; k < LASER_RANGE_ARRAY_NUM_SENSORS; k++) {
    struct FloatEulers def = {laser_range_array_orientations[k * 3], laser_range_array_orientations[k * 3 + 1],
        laser_range_array_orientations[k * 3 + 2]};
    struct FloatQuat q_def;
    float_quat_of_eulers(&q_def, &def);
    // get angle between required angle and ray angle
    float angle = acosf(QUAT_DOT_PRODUCT(q_down, q_def));

    if (fabsf(angle) < RadOfDeg(5)) {
      agl_id = k;
      break;
    }
  }
#endif

#if LASER_RANGE_ARRAY_SEND_FRONT_OBSTACLE
  // Determine which sensor is looking down
  struct FloatEulers pose_forward = {0., 0., 0.};

  struct FloatQuat q_forward;
  float_quat_of_eulers(&q_forward, &pose_forward);

  for (int k = 0; k < LASER_RANGE_ARRAY_NUM_SENSORS; k++) {
    struct FloatEulers def = {laser_range_array_orientations[k * 3], laser_range_array_orientations[k * 3 + 1],
        laser_range_array_orientations[k * 3 + 2]};
    struct FloatQuat q_def;
    float_quat_of_eulers(&q_def, &def);
    // get angle between required angle and ray angle
    float angle = acosf(QUAT_DOT_PRODUCT(q_forward, q_def));

    if (fabsf(angle) < RadOfDeg(5)) {
      front_id = k;
      break;
    }
  }
#endif
}

/* Parse the InterMCU message */
static void laser_range_array_parse_msg(void)
{
  uint8_t msg_id = lra_msg_buf[1];

  // Get Time of Flight laser range sensor ring  messages
  switch (msg_id) {
    case DL_IMCU_REMOTE_GROUND: {
      uint8_t id = DL_IMCU_REMOTE_GROUND_id(lra_msg_buf);
      uint16_t range = DL_IMCU_REMOTE_GROUND_range(lra_msg_buf);

      if (id < LASER_RANGE_ARRAY_NUM_SENSORS) {
        laser_range_array_values[id] = range;
        id_recieved[id] = 1;

        // check how many sensors received
        int16_t sum = 0;
        for (uint8_t i = 0; i < LASER_RANGE_ARRAY_NUM_SENSORS; i++)
        {
          sum += id_recieved[id];
        }

        if(sum == LASER_RANGE_ARRAY_NUM_SENSORS)
        {
          // wait till all sensors received before sending update
          AbiSendMsgRANGE_SENSOR_ARRAY(RANGE_SENSOR_ARRAY_VL53L0_ID, LASER_RANGE_ARRAY_NUM_SENSORS,
              laser_range_array_values, laser_range_array_orientations);
          // reset id array
          memset(id_recieved, 0, sizeof(id_recieved));
        }

        if (id == agl_id) {
          float agl = (float)range / 1000.;
          AbiSendMsgAGL(AGL_VL53L0_LASER_ARRAY_ID, agl);
        }
        if (id == front_id) {
          float dist = (float)range / 1000.;
          // todo create distance estimate sender ids
          AbiSendMsgOBSTACLE_DETECTION(RANGE_SENSOR_ARRAY_VL53L0_ID, dist, 0.);
        }
      }
      break;
    }
    default:
      break;
  }
}

void laser_range_array_event(void)
{
  pprz_check_and_parse(laser_range_array.device, &laser_range_array.transport, lra_msg_buf,
      &laser_range_array.msg_available);

  if (laser_range_array.msg_available) {
    laser_range_array_parse_msg();
    laser_range_array.msg_available = false;
  }
}
