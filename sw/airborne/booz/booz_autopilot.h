/*
 * $Id$
 *  
 * Copyright (C) 2008  Antoine Drouin
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
 *
 */

#ifndef BOOZ_AUTOPILOT_H
#define BOOZ_AUTOPILOT_H

#include "std.h"

#define BOOZ_AP_MODE_FAILSAFE     0
#define BOOZ_AP_MODE_KILL         1
#define BOOZ_AP_MODE_RATE         2
#define BOOZ_AP_MODE_ATTITUDE     3
#define BOOZ_AP_MODE_HEADING_HOLD 4
#define BOOZ_AP_MODE_NAV          5

extern uint8_t booz_autopilot_mode;

extern void booz_autopilot_init(void);
extern void booz_autopilot_periodic_task(void);
extern void booz_autopilot_on_rc_event(void);

#define TRESHOLD_RATE_PPRZ (MIN_PPRZ / 2)
#define TRESHOLD_ATTITUDE_PPRZ  (MAX_PPRZ/2)
#define BOOZ_AP_MODE_OF_PPRZ(rc)					\
  ((rc) < TRESHOLD_RATE_PPRZ ? BOOZ_AP_MODE_ATTITUDE :			\
   (rc) < TRESHOLD_ATTITUDE_PPRZ ? BOOZ_AP_MODE_HEADING_HOLD :		\
   BOOZ_AP_MODE_NAV )

#endif /* BOOZ_AUTOPILOT_H */
