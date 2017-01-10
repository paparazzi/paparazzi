/*
 *
 * Copyright (C) 2016, Michal Podhradsky
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

/**
 * @file modules/nav/nav_launcher.h
 * @brief Pneumatic launcher system
 * See video of the system: https://www.youtube.com/watch?v=qc1uwH-8Dbw
 *       Launcher.
 *    A pneumatic launching system.
 *    - Phase 1: Zero Roll, Takeoff Pitch, Full Throttle(once you enter the block!)
 *    - Phase 2: After detecting lauch (ground speed) and travelling enough distance from the launch point
 *               follow launch line -> Auto roll, Takeoff pitch,  Full Throttle
 *    - Phase 3: If the aircraft is above a specific alt, greater than a specific speed or too far away, circle up
 *               with takeoff circle radius, until you reach desired takeoff altitude
 *
 *   An example section to be added into your airframe configuration:
 *   <!-- Launcher Takeoff Configuration -->
 *   <section name="LAUNCHER" prefix="LAUNCHER_TAKEOFF_">
 *     <define name="PITCH" value="0.23" unit="rad"/>
 *     <define name="HEIGH" value="70" unit="m"/>
 *     <define name="MIN_SPEED_CIRCLE" value="8" unit="m/s"/>
 *     <define name="DISTANCE" value="30" unit="m"/>
 *     <define name="MIN_SPEED_LINE" value="5" unit="m/s"/>
 *   </section>
 */

#ifndef NAV_LAUNCHER_H
#define NAV_LAUNCHER_H

#include "std.h"
#include "paparazzi.h"

extern void nav_launcher_setup(void);
extern bool nav_launcher_run(void);

#endif /* NAV_LAUNCHER_H */
