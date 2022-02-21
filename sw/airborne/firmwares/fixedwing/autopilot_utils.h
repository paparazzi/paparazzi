/*
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 */

/**
 * @file firmwares/fixedwing/autopilot_utils.h
 *
 * Utility functions and includes for autopilots
 *
 */

#ifndef AUTOPILOT_UTILS_H
#define AUTOPILOT_UTILS_H

#include "std.h"
#include "generated/airframe.h"

/** Get mode from pulse
 */
#define AP_MODE_OF_PULSE(pprz) \
  (pprz > THRESHOLD2 ? AP_MODE_AUTO2 : \
   (pprz > THRESHOLD1 ? AP_MODE_AUTO1 : AP_MODE_MANUAL))

/** Stick pushed
 */
#define STICK_PUSHED(pprz) (pprz < THRESHOLD1 || pprz > THRESHOLD2)

/** pprz_t to float with saturation
 */
#define FLOAT_OF_PPRZ(pprz, center, travel) ((float)pprz / (float)MAX_PPRZ * travel + center)

/** Takeoff detection threshold from throttle
 */
#define THROTTLE_THRESHOLD_TAKEOFF (pprz_t)(MAX_PPRZ * 0.9)

/** Threshold for RC mode detection.
 */
#define THRESHOLD_MANUAL_PPRZ (MIN_PPRZ / 2)
#define THRESHOLD1 THRESHOLD_MANUAL_PPRZ
#define THRESHOLD2 (MAX_PPRZ/2)

/** AP command setter macros for usual commands
 */

// COMMAND_ROLL
#define AP_COMMAND_SET_ROLL(_roll) { command_set(COMMAND_ROLL, _roll); }

// COMMAND_PITCH
#define AP_COMMAND_SET_PITCH(_pitch) { command_set(COMMAND_PITCH, _pitch); }

// COMMAND_YAW
#if H_CTL_YAW_LOOP && defined COMMAND_YAW
#define AP_COMMAND_SET_YAW(_yaw) { command_set(COMMAND_YAW, _yaw); }
#else
#define AP_COMMAND_SET_YAW(_yaw) {}
#endif

// COMMAND_THROTTLE
#define AP_COMMAND_SET_THROTTLE(_throttle) { \
  command_set(COMMAND_THROTTLE, _throttle); \
  autopilot.throttle = _throttle; \
}

// COMMAND_CL
#if H_CTL_CL_LOOP && defined COMMAND_CL
#define AP_COMMAND_SET_CL(_cl) { command_set(COMMAND_CL, _cl); }
#else
#define AP_COMMAND_SET_CL(_cl) {}
#endif

// ROLL setpoint from RADIO
#define AP_SETPOINT_ROLL(_roll, _max) { \
  _roll = FLOAT_OF_PPRZ(radio_control_get(RADIO_ROLL), 0., _max); \
}

// PITCH setpoint from RADIO
#define AP_SETPOINT_PITCH(_pitch, _max) { \
  _pitch = FLOAT_OF_PPRZ(radio_control_get(RADIO_PITCH), 0., _max); \
}

// PITCH setpoint from RADIO
#if H_CTL_YAW_LOOP && defined RADIO_YAW
#define AP_SETPOINT_YAW_RATE(_yaw, _max) { \
  _yaw = FLOAT_OF_PPRZ(radio_control_get(RADIO_YAW), 0., _max); \
}
#else
#define AP_SETPOINT_YAW_RATE(_yaw, _max) {}
#endif

// THROTTLE setpoint from RADIO
#define AP_SETPOINT_THROTTLE(_throttle) { \
  _throttle = radio_control_get(RADIO_THROTTLE); \
}


#endif // AUTOPILOT_UTILS_H

