/*
 * Copyright (C) 2020 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file firmwares/rotorcraft/autopilot_arming.h
 *
 * Arming procedure for rotorcraft
 * Several options can be selected:
 *  - yaw stick
 *  - switch position
 *  - throttle stick
 *
 */

#ifndef AUTOPILOT_ARMING_H
#define AUTOPILOT_ARMING_H

/* Include arming procedure, yaw stick by default */
#if USE_KILL_SWITCH_FOR_MOTOR_ARMING
#include "autopilot_arming_switch.h"
PRINT_CONFIG_MSG("Using kill switch for motor arming")
#elif USE_THROTTLE_FOR_MOTOR_ARMING
#include "autopilot_arming_throttle.h"
PRINT_CONFIG_MSG("Using throttle for motor arming")
#else
#include "autopilot_arming_yaw.h"
PRINT_CONFIG_MSG("Using 2 sec yaw for motor arming")
#endif

#endif /* AUTOPILOT_ARMING_H */
