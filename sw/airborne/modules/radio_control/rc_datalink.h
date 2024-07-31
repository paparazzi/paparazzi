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

#define RC_DL_NB_CHANNEL 11

/**
 * Redefining RADIO_*
 * Do not use with radio.h (ppm rc)
 */
#define RADIO_ROLL      0  // Default: Roll 
#define RADIO_PITCH     1  // Default: Pitch
#define RADIO_YAW       2  // Default: Yaw
#define RADIO_THROTTLE  3  // Default: Thrust
#define RADIO_MODE      4  // Default: MODE (MANUAL / AUTO)
#define RADIO_AUX1      5  // Default: KILL
#define RADIO_AUX2      7  // Default: NA
#define RADIO_AUX4      8  // Default: NA
#define RADIO_AUX5      9  // Default: NA
#define RADIO_AUX6      10 // Default: NA
#define RADIO_AUX7      6  // Default: AP_MODE (EXTRA MODE SWITCH: e.g Switch NAV from INDI to ANDI to PID)

#ifndef RADIO_KILL_SWITCH
#define RADIO_KILL_SWITCH RADIO_AUX1
#endif

#ifndef AP_MODE_SWITCH
#define AP_MODE_SWITCH RADIO_AUX7
#endif

extern int8_t rc_dl_values[RC_DL_NB_CHANNEL];
extern volatile bool rc_dl_frame_available;

/**
 * Decode datalink message to get rc values with RC_UP message
 */
extern void parse_rc_up_datalink(
    int8_t n, int8_t *channels);    

extern void rc_datalink_parse_RC_UP(uint8_t *buf);
/**
 * RC init function.
 */
extern void rc_datalink_init(void);

/**
 * RC event function.
 */
extern void rc_datalink_event(void);

#endif /* RC_DATALINK_H */
