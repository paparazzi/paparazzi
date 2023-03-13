/*
 * Copyright (C) 2022 Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
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

/** @file "modules/ctrl/eff_scheduling_nederdrone.h"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * Interpolation of control effectivenss matrix
of the Nederdrone.

If instead using online adaptation is an option, be sure to
not use this module at the same time!
 */

#ifndef EFF_SCHEDULING_NEDERDRONE_H
#define EFF_SCHEDULING_NEDERDRONE_H

#include <stdio.h>
#include <stdbool.h>

extern void ctrl_eff_scheduling_init(void);
extern void ctrl_eff_scheduling_periodic(void);

// Functions to schedule switching on and of of tip props on front wing
extern float sched_ratio_tip_props;
// If pitch lower, pitch props gradually switch off till  sched_tip_prop_lower_pitch_limit_deg (1 > sched_ratio_tip_props > 0)
extern float sched_tip_prop_upper_pitch_limit_deg;
// If pitch lower, pitch props switch fully off (sched_ratio_tip_props goes to 0)
extern float sched_tip_prop_lower_pitch_limit_deg;
// Setting to not switch off tip props during forward flight
extern bool sched_tip_props_always_on;
// Setting to scale the thrust effectiveness
extern float thrust_eff_scaling;

// Schedule all forward actuators with airspeed instead of only the flaps
extern bool all_act_fwd_sched;

// trim aerodynamic surfaces with settings
extern float trim_elevator;
extern float trim_flaps;

#endif  // EFF_SCHEDULING_NEDERDRONE_H
