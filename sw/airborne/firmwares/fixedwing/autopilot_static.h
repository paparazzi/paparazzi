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
#define  AP_MODE_MANUAL 0
#define  AP_MODE_AUTO1 1
#define  AP_MODE_AUTO2 2
#define  AP_MODE_HOME 3
#define  AP_MODE_GPS_OUT_OF_ORDER 4
#define  AP_MODE_NB 5

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


#ifndef CONTROL_FREQUENCY
#define CONTROL_FREQUENCY 60
#endif

#endif /* AUTOPILOT_STATIC_H */

