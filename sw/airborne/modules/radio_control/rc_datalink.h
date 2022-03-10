/*
 * Copyright (C) 2010-2014 The Paparazzi Team
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
 * @file modules/radio_control/rc_datalink.h
 *
 * Radio control input via datalink.
 */

#ifndef RC_DATALINK_H
#define RC_DATALINK_H

#include "std.h"

#define RC_DL_NB_CHANNEL 5

/**
 * Redefining RADIO_*
 * Do not use with radio.h (ppm rc)
 */
#define RADIO_ROLL      0
#define RADIO_PITCH     1
#define RADIO_YAW       2
#define RADIO_THROTTLE  3
#define RADIO_MODE      4

extern int8_t rc_dl_values[ RC_DL_NB_CHANNEL ];
extern volatile bool rc_dl_frame_available;

/**
 * Decode datalink message to get rc values with RC_3CH message
 * Mode and throttle are merge in the same byte
 */
extern void parse_rc_3ch_datalink(
  uint8_t throttle_mode,
  int8_t roll,
  int8_t pitch);

/**
 * Decode datalink message to get rc values with RC_4CH message
 */
extern void parse_rc_4ch_datalink(
  uint8_t mode,
  uint8_t throttle,
  int8_t roll,
  int8_t pitch,
  int8_t yaw);

extern void rc_datalink_parse_RC_3CH(uint8_t *buf);
extern void rc_datalink_parse_RC_4CH(uint8_t *buf);

/**
 * RC init function.
 */
extern void rc_datalink_init(void);

/**
 * RC event function.
 */
extern void rc_datalink_event(void);

#endif /* RC_DATALINK_H */

