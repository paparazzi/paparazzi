/*
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/datalink/bitcraze/syslink.h
 *
 * Syslink protocol for communication with bitcraze/crazyflie NRF mcu
 *
 * based on PX4 implementation
 */

#ifndef SYSLINK_H
#define SYSLINK_H

#include <stdint.h>
#include <stdbool.h>

#define SYSLINK_GROUP                 0xF0

#define SYSLINK_RADIO                 0x00
#define SYSLINK_RADIO_RAW             0x00
#define SYSLINK_RADIO_CHANNEL         0x01
#define SYSLINK_RADIO_DATARATE        0x02
#define SYSLINK_RADIO_CONTWAVE        0x03
#define SYSLINK_RADIO_RSSI            0x04
#define SYSLINK_RADIO_ADDRESS         0x05

#define SYSLINK_PM                    0x10
#define SYSLINK_PM_SOURCE             0x10
#define SYSLINK_PM_ONOFF_SWITCHOFF    0x11
#define SYSLINK_PM_BATTERY_VOLTAGE    0x12
#define SYSLINK_PM_BATTERY_STATE      0x13
#define SYSLINK_PM_BATTERY_AUTOUPDATE 0x14

#define SYSLINK_OW                    0x20
#define SYSLINK_OW_SCAN               0x20
#define SYSLINK_OW_GETINFO            0x21
#define SYSLINK_OW_READ               0x22
#define SYSLINK_OW_WRITE              0x23

// Limited by the CRTP packet which is limited by the ESB protocol used by the NRF
#define SYSLINK_MAX_DATA_LEN    32

#define SYSLINK_RADIO_RATE_250K 0
#define SYSLINK_RADIO_RATE_1M   1
#define SYSLINK_RADIO_RATE_2M   2


typedef struct {
  uint8_t type;
  uint8_t length;
  uint8_t data[SYSLINK_MAX_DATA_LEN];
  uint8_t cksum[2];
} __attribute__((packed)) syslink_message_t;


typedef enum {
  SYSLINK_STATE_START = 0,
  SYSLINK_STATE_TYPE,
  SYSLINK_STATE_LENGTH,
  SYSLINK_STATE_DATA,
  SYSLINK_STATE_CKSUM
} syslink_state_t;

typedef struct  {
  syslink_state_t state;
  int index;
} syslink_parse_state;

extern const char *syslink_stx;

#ifdef __cplusplus
extern "C" {
#endif

  /** Init syslink parser
   *
   * @param state pointer to state structure
   */
  extern void syslink_parse_init(syslink_parse_state *state);

  /** Parse one byte
   *
   * @param state pointer to state structure
   * @param c next byte to parse
   * @param msg pointer to message structure
   * @return true if a full message was parsed
   */
  extern bool syslink_parse_char(syslink_parse_state *state, uint8_t c, syslink_message_t *msg);

  /** Compute syslink checksum
   *
   * @param msg pointer to message structure
   */
  extern void syslink_compute_cksum(syslink_message_t *msg);

#ifdef __cplusplus
}
#endif

#endif

