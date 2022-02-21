/*
 * Copyright (C) 2015 The Paparazzi Team
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
 *
 */

/**
 * @file main_fbw.h
 *
 * Fly By Wire:
 *
 * Reads radio_control
 * Reads intermcu
 * Sets actuators
 * Run datalink/telemetry
 *
 * if no rc but autopilot then RC_LOST_FBW_MODE (define below)
 * if no rc while in auto mode then RC_LOST_IN_AUTO_FBW_MODE (define below)
 * if no ap but rc then AP_LOST_FBW_MODE (define below)
 */

#ifndef MAIN_FBW_H
#define MAIN_FBW_H

/** mode to enter when RC is lost while using a mode with RC input
 *  switching to AUTO allows a recover with HOME mode
 */
#ifndef RC_LOST_FBW_MODE
#define RC_LOST_FBW_MODE FBW_MODE_AUTO
#endif

/** mode to enter when AP is lost while using autopilot */
#ifndef RC_LOST_IN_AUTO_FBW_MODE
#define RC_LOST_IN_AUTO_FBW_MODE FBW_MODE_AUTO
#endif

/** mode to enter when AP is lost while using autopilot */
#ifndef AP_LOST_FBW_MODE
#define AP_LOST_FBW_MODE FBW_MODE_FAILSAFE
#endif

/** holds whether the aircraft can only be flown with the AP and not RC-Direct/FBW-mode */
#ifndef FBW_MODE_AUTO_ONLY
#define FBW_MODE_AUTO_ONLY false
#endif

/** Switching between FBW and autopilot is done with RADIO_FBW_MODE: default is to re-use RADIO_MODE */
#ifndef RADIO_FBW_MODE
#define RADIO_FBW_MODE RADIO_MODE
#endif

#define FBW_MODE_MANUAL   0
#define FBW_MODE_AUTO     1
#define FBW_MODE_FAILSAFE 2

#include "std.h"

extern uint8_t fbw_mode;
extern bool fbw_motors_on;

extern void main_fbw_init(void);
extern void main_fbw_event(void);
extern void main_fbw_periodic(void);
extern void main_fbw_parse_EMERGENCY_CMD(uint8_t *buf);

#endif /* MAIN_FBW_H */

