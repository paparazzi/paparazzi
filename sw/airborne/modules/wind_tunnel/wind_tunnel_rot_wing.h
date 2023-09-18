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

/** @file "modules/wind_tunnel/wind_tunnel_rot_wing.h"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * This module allows the user to control seperate actuators for example during wind tunnel experiments.
 */

#ifndef WIND_TUNNEL_ROT_WING_H
#define WIND_TUNNEL_ROT_WING_H

#include "std.h"
#include "stdbool.h"

// define variables
extern int16_t actuators_wt[11];
extern int16_t actuators_slider_wt[11];
extern int16_t motors_slider_wt;

extern bool motors_on_wt;
extern bool motor_on_wt[5];

// actuator sweep parameters
extern uint8_t wt_actuator_sweep_index;
extern int16_t wt_input_min_cmd;
extern int16_t wt_input_max_cmd;
extern float wt_input_steptime;

// status indicators
extern bool wt_sweep_running;
extern bool wt_sweep_motors_running;

extern void init_wt_rot_wing(void);
extern void event_wt_rot_wing(void);

extern void wind_tunnel_rot_wing_sweep_handler(bool activate);
extern void wind_tunnel_rot_wing_sweep_motors_handler(bool activate);

#endif  // WIND_TUNNEL_ROT_WING_H
