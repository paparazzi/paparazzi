/*
 * Copyright (C) 2022 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file modules/radio_control/rc_intermcu.h
 *
 * Radio control input via intermcu.
 */

#ifndef RC_INTERMCU_H
#define RC_INTERMCU_H

#include "std.h"

/* We need radio defines for the Autopilot
 * index are matching intermcu_fbw.h
 */
#define RADIO_THROTTLE    0
#define RADIO_ROLL        1
#define RADIO_PITCH       2
#define RADIO_YAW         3
#define RADIO_MODE        4
#define RADIO_KILL_SWITCH 5
#define RADIO_AUX1        5 // be careful with ID 5
#define RADIO_AUX2        6
#define RADIO_AUX3        7

#define RC_IMCU_NB_CHANNEL  8


/**
 * RC init function.
 */
extern void rc_intermcu_init(void);

/**
 * Decode intermcu message to get rc values
 * and FBW status for RC status and frame rate TODO make a single message
 */
extern void rc_intermcu_parse_msg(uint8_t *buf);
extern void rc_intermcu_parse_fbw_status(uint8_t *buf);

#endif /* RC_INTERMCU_H */

