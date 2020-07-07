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
 * @file firmwares/rotorcraft/autopilot_arming_common.h
 *
 * Arming procedure for rotorcraft, common definitions
 */

#ifndef AUTOPILOT_ARMING_COMMON_H
#define AUTOPILOT_ARMING_COMMON_H

#include "autopilot_rc_helpers.h"

/* Arming status*/
#define AP_ARMING_STATUS_NO_RC                0
#define AP_ARMING_STATUS_WAITING              1
#define AP_ARMING_STATUS_ARMING               2
#define AP_ARMING_STATUS_ARMED                3
#define AP_ARMING_STATUS_DISARMING            4
#define AP_ARMING_STATUS_KILLED               5
#define AP_ARMING_STATUS_YAW_CENTERED         6
#define AP_ARMING_STATUS_THROTTLE_DOWN        7
#define AP_ARMING_STATUS_NOT_MODE_MANUAL      8
#define AP_ARMING_STATUS_UNARMED_IN_AUTO      9
#define AP_ARMING_STATUS_THROTTLE_NOT_DOWN    10
#define AP_ARMING_STATUS_STICKS_NOT_CENTERED  11
#define AP_ARMING_STATUS_PITCH_NOT_CENTERED   12
#define AP_ARMING_STATUS_ROLL_NOT_CENTERED    13
#define AP_ARMING_STATUS_YAW_NOT_CENTERED     14
#define AP_ARMING_STATUS_AHRS_NOT_ALLIGNED    15
#define AP_ARMING_STATUS_OUT_OF_GEOFENCE      16
#define AP_ARMING_STATUS_LOW_BATTERY          17

#endif /* AUTOPILOT_ARMING_COMMON_H */
