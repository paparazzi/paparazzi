/*
 * Copyright (C) 2019 Freek van Tienen <freek.v.tienen@gmail.com>
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

/** @file modules/lidar/tfmini.c
 *  @brief driver for the TFMini lidar
 *
 */
#include "tfmini.h"
#include "mcu_periph/uart.h"
#include "subsystems/abi.h"

// State interface for rotation compensation
#include "state.h"

// Messages
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

struct TFMini tfmini = {
  .parse_status = TFMINI_INITIALIZE
};

static void tfmini_parse(uint8_t byte);

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

/**
 * Downlink message lidar
 */
static void tfmini_send_lidar(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t status = (uint8_t) tfmini.parse_status;
  pprz_msg_send_LIDAR(trans, dev, AC_ID,
                      &tfmini.distance,
                      &tfmini.mode,
                      &status);
}

#endif

/**
 * Initialization function
 */
void tfmini_init(void)
{
  tfmini.device = &((TFMINI_PORT).device);

  tfmini.update_agl = USE_TFMINI_AGL;
  tfmini.compensate_rotation = TFMINI_COMPENSATE_ROTATION;

  tfmini.strength = 0;
  tfmini.distance = 0;
  tfmini.parse_status = TFMINI_PARSE_HEAD;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_LIDAR, tfmini_send_lidar);
#endif
}

/**
 * Lidar event function
 * Receive bytes from the UART port and parse them
 */
void tfmini_event(void)
{
  while (tfmini.parse_status != TFMINI_INITIALIZE && tfmini.device->char_available(tfmini.device->periph)) {
    tfmini_parse(tfmini.device->get_byte(tfmini.device->periph));
  }
}

/**
 * Parse the lidar bytes 1 by 1
 */
static void tfmini_parse(uint8_t byte)
{
  switch (tfmini.parse_status) {
    case TFMINI_INITIALIZE:
      break;
    case TFMINI_PARSE_HEAD:
      if (byte == 0x59) {
        tfmini.parse_crc = byte;
        tfmini.parse_status++;
      }
      break;
    case TFMINI_PARSE_HEAD2:
      if (byte == 0x59) {
        tfmini.parse_crc += byte;
        tfmini.parse_status++;
      } else {
        tfmini.parse_status = TFMINI_PARSE_HEAD;
      }
      break;

    case TFMINI_PARSE_DIST_L:
      tfmini.raw_dist = byte;
      tfmini.parse_crc += byte;
      tfmini.parse_status++;
      break;
    case TFMINI_PARSE_DIST_H:
      tfmini.raw_dist |= (byte << 8);
      tfmini.parse_crc += byte;
      tfmini.parse_status++;
      break;

    case TFMINI_PARSE_STRENGTH_L:
      tfmini.raw_strength = byte;
      tfmini.parse_crc += byte;
      tfmini.parse_status++;
      break;
    case TFMINI_PARSE_STRENGTH_H:
      tfmini.raw_strength |= (byte << 8);
      tfmini.parse_crc += byte;
      tfmini.parse_status++;
      break;

    case TFMINI_PARSE_MODE:
      tfmini.raw_mode = byte;
      tfmini.parse_crc += byte;
      tfmini.parse_status++;
      break;
    case TFMINI_PARSE_BYTE7:
      tfmini.parse_crc += byte;
      tfmini.parse_status++;
      break;

    case TFMINI_PARSE_CHECKSUM:
      // When the CRC matches
      if (tfmini.parse_crc == byte) {
        uint32_t now_ts = get_sys_time_usec();
        tfmini.distance = tfmini.raw_dist / 100.f;
        tfmini.strength = tfmini.raw_strength;
        tfmini.mode = tfmini.raw_mode;

        // When the distance is valid
        if (tfmini.distance != 0xFFFF) {
          // compensate AGL measurement for body rotation
          if (tfmini.compensate_rotation) {
            float phi = stateGetNedToBodyEulers_f()->phi;
            float theta = stateGetNedToBodyEulers_f()->theta;
            float gain = (float)fabs((double)(cosf(phi) * cosf(theta)));
            tfmini.distance = tfmini.distance * gain;
          }

          // send message (if requested)
          if (tfmini.update_agl) {
            AbiSendMsgAGL(AGL_LIDAR_TFMINI_ID, now_ts, tfmini.distance);
          }
        }
      }

      // Start reading again
      tfmini.parse_status = TFMINI_PARSE_HEAD;
      break;

    default:
      // Error, return to start
      tfmini.parse_status = TFMINI_PARSE_HEAD;
      break;
  }
}
