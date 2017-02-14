/*
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file firmwares/fixedwing/autopilot_static.h
 *
 * Fixedwing autopilot modes (static implementation).
 *
 */

#ifndef AUTOPILOT_STATIC_H
#define AUTOPILOT_STATIC_H

#include "autopilot.h"

/** AP modes.
 */
#define  PPRZ_MODE_MANUAL 0
#define  PPRZ_MODE_AUTO1 1
#define  PPRZ_MODE_AUTO2 2
#define  PPRZ_MODE_HOME 3
#define  PPRZ_MODE_GPS_OUT_OF_ORDER 4
#define  PPRZ_MODE_NB 5

/** Static autopilot functions
 */
extern void autopilot_static_init(void);
extern void autopilot_static_periodic(void);
extern void autopilot_static_on_rc_frame(void);
extern void autopilot_static_set_mode(uint8_t new_autopilot_mode);
extern void autopilot_static_SetModeHandler(float new_autopilot_mode); // handler for dl_setting
extern void autopilot_static_set_motors_on(bool motors_on);

/** Control loops
 * FIXME should be somewhere else
 */
extern void navigation_task(void);
extern void attitude_loop(void);

/** Threshold for RC mode detection.
 */
#define THRESHOLD_MANUAL_PPRZ (MIN_PPRZ / 2)
#define THRESHOLD1 THRESHOLD_MANUAL_PPRZ
#define THRESHOLD2 (MAX_PPRZ/2)

#define PPRZ_MODE_OF_PULSE(pprz) \
  (pprz > THRESHOLD2 ? PPRZ_MODE_AUTO2 : \
   (pprz > THRESHOLD1 ? PPRZ_MODE_AUTO1 : PPRZ_MODE_MANUAL))


// FIXME, move to control
#define LATERAL_MODE_MANUAL    0
#define LATERAL_MODE_ROLL_RATE 1
#define LATERAL_MODE_ROLL      2
#define LATERAL_MODE_COURSE    3
#define LATERAL_MODE_NB        4
extern uint8_t lateral_mode;

#define STICK_PUSHED(pprz) (pprz < THRESHOLD1 || pprz > THRESHOLD2)
#define FLOAT_OF_PPRZ(pprz, center, travel) ((float)pprz / (float)MAX_PPRZ * travel + center)

#define THROTTLE_THRESHOLD_TAKEOFF (pprz_t)(MAX_PPRZ * 0.9)

/* CONTROL_RATE will be removed in the next release
 * please use CONTROL_FREQUENCY instead
 */
#ifndef CONTROL_FREQUENCY
#ifdef  CONTROL_RATE
#define CONTROL_FREQUENCY CONTROL_RATE
#warning "CONTROL_RATE is deprecated. Please use CONTROL_FREQUENCY instead. Defaults to 60Hz if not defined."
#else
#define CONTROL_FREQUENCY 60
#endif  // CONTROL_RATE
#endif  // CONTROL_FREQUENCY

#ifndef NAVIGATION_FREQUENCY
#define NAVIGATION_FREQUENCY 4
#endif

#endif /* AUTOPILOT_STATIC_H */

