/*
 * Copyright (C) 2015 The Paparazzi Team
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
 *
 */

/** @file modules/intermcu/intermcu_fbw.h
 *  @brief Inter-MCU on FlyByWire side
 */

#ifndef INTERMCU_FBW_H
#define INTERMCU_FBW_H

#include "modules/intermcu/intermcu.h"
#include "pprzlink/intermcu_msg.h"

extern bool intermcu_ap_motors_on;
extern pprz_t intermcu_commands[COMMANDS_NB];

/** send fbw status message
 */
extern void intermcu_send_status(void);

/** Datalink event functions
 */
extern void intermcu_parse_IMCU_COMMANDS(uint8_t *buf);
extern void intermcu_parse_IMCU_SPEKTRUM_SOFT_BIND(uint8_t *buf);
extern void intermcu_forward_uplink(uint8_t *buf);


/* We need radio defines for the Autopilot
 * index are matching  rc_intermcu.h
 */
#define INTERMCU_RADIO_THROTTLE     0
#define INTERMCU_RADIO_ROLL         1
#define INTERMCU_RADIO_PITCH        2
#define INTERMCU_RADIO_YAW          3
#define INTERMCU_RADIO_MODE         4
#define INTERMCU_RADIO_KILL_SWITCH  5
#define INTERMCU_RADIO_AUX1         5 // be careful with ID 5
#define INTERMCU_RADIO_AUX2         6
#define INTERMCU_RADIO_AUX3         7

#define INTERMCU_RADIO_CONTROL_NB_CHANNEL 8

#endif /* INTERMCU_FB_H */

