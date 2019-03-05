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

#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

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
static float laser_range_array_orientations[] = LASER_RANGE_ARRAY_ORIENTATIONS;
static uint8_t agl_id = 255;

#define VL53L0_MAX_VAL 8.191f

void laser_range_array_init(void)
{
  // Initialize transport protocol
  pprz_transport_init(&laser_range_array.transport);

#if LASER_RANGE_ARRAY_SEND_AGL
  // find sensor looking down
  for (int k = 0; k < LASER_RANGE_ARRAY_NUM_SENSORS; k++) {
    if (fabsf(laser_range_array_orientations[k * 2] + M_PI_2) < RadOfDeg(5)) {
      agl_id = k;
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
      uint32_t now_ts = get_sys_time_usec();
      uint8_t id = DL_IMCU_REMOTE_GROUND_id(lra_msg_buf);

      if (id < LASER_RANGE_ARRAY_NUM_SENSORS) {
        float range = DL_IMCU_REMOTE_GROUND_range(lra_msg_buf) / 1000.f;
        // wait till all sensors received before sending update
        AbiSendMsgOBSTACLE_DETECTION(OBS_DETECTION_RANGE_ARRAY_ID, range, laser_range_array_orientations[id*2],
            laser_range_array_orientations[id*2 + 1]);

        if (id == agl_id && range > 1e-5 && range < VL53L0_MAX_VAL) {
          AbiSendMsgAGL(AGL_VL53L0_LASER_ARRAY_ID, now_ts, range);
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
