/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 * Copyright (C) 2016 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file firmwares/rotorcraft/autopilot_static.h
 *
 * Autopilot static implementation
 *
 */

#ifndef AUTOPILOT_STATIC_H
#define AUTOPILOT_STATIC_H

/** Static autopilot modes
 */
#define AP_MODE_KILL              0
#define AP_MODE_FAILSAFE          1
#define AP_MODE_HOME              2
#define AP_MODE_RATE_DIRECT       3
#define AP_MODE_ATTITUDE_DIRECT   4
#define AP_MODE_RATE_RC_CLIMB     5
#define AP_MODE_ATTITUDE_RC_CLIMB 6
#define AP_MODE_ATTITUDE_CLIMB    7
#define AP_MODE_RATE_Z_HOLD       8
#define AP_MODE_ATTITUDE_Z_HOLD   9
#define AP_MODE_HOVER_DIRECT      10
#define AP_MODE_HOVER_CLIMB       11
#define AP_MODE_HOVER_Z_HOLD      12
#define AP_MODE_NAV               13
#define AP_MODE_RC_DIRECT         14  // Safety Pilot Direct Commands for helicopter direct control
#define AP_MODE_CARE_FREE_DIRECT  15
#define AP_MODE_FORWARD           16
#define AP_MODE_MODULE            17
#define AP_MODE_FLIP              18
#define AP_MODE_GUIDED            19


/** Default RC mode.
 */
#ifndef MODE_MANUAL
#define MODE_MANUAL AP_MODE_ATTITUDE_DIRECT
#endif
#ifndef MODE_AUTO1
#define MODE_AUTO1 AP_MODE_HOVER_Z_HOLD
#endif
#ifndef MODE_AUTO2
#define MODE_AUTO2 AP_MODE_NAV
#endif

/** Specific function for static AP
 */
extern void autopilot_static_init(void);
extern void autopilot_static_periodic(void);
extern void autopilot_static_on_rc_frame(void);
extern void autopilot_static_set_mode(uint8_t new_autopilot_mode);
extern void autopilot_static_SetModeHandler(float new_autopilot_mode); // handler for dl_setting
extern void autopilot_static_set_motors_on(bool motors_on);

#endif /* AUTOPILOT_STATIC_H */
