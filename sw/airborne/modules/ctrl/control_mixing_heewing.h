/*
 * Copyright (C) 2024 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi
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

/** @file "modules/ctrl/control_mixing_heewing.h"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Control mixing specific to the Heewing T1 Ranger
 */

#ifndef CONTROL_MIXING_HEEWING_H
#define CONTROL_MIXING_HEEWING_H

// INDI actuators output indexes
#define CMH_ACT_MOTOR_RIGHT 0
#define CMH_ACT_MOTOR_LEFT  1
#define CMH_ACT_MOTOR_TAIL  2
#define CMH_ACT_YAW         3

extern void control_mixing_heewing_init(void);

/** Direct manual control in plane style flight
 */
extern void control_mixing_heewing_manual(void);

/** Stabilization in attitude direct mode
 */
extern void control_mixing_heewing_attitude_direct(void);
extern void control_mixing_heewing_attitude_direct_enter(void);
extern void control_mixing_heewing_attitude_plane(void);
extern void control_mixing_heewing_attitude_plane_enter(void);
extern void control_mixing_heewing_nav_enter(void);
extern void control_mixing_heewing_nav_run(void);

#endif  // CONTROL_MIXING_HEEWING_H
