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

/** @file "modules/rot_wing_drone/wing_rotation_controller_servo.h"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * Module to control wing rotation servo command based on prefered angle setpoint
 */

#ifndef WING_ROTATION_CONTROLLER_SERVO_H
#define WING_ROTATION_CONTROLLER_SERVO_H

#include "std.h"

extern void wing_rotation_init(void);
extern void wing_rotation_periodic(void);
extern void wing_rotation_event(void);
extern bool set_wing_rotation_scheduler(bool rotation_scheduler_on);
extern bool set_wing_rotation_scheduler_nav(bool rotation_scheduler_on);

// Paramaters
struct wing_rotation_controller {
  int32_t servo_pprz_cmd;
  uint16_t adc_wing_rotation;
  int16_t adc_wing_rotation_range;
  float wing_angle_rad;
  float wing_angle_deg;
  float wing_angle_rad_sp;
  float wing_angle_deg_sp;
  float wing_rotation_speed;
  float wing_angle_virtual_deg_sp;
  float wing_rotation_first_order_dynamics;
  float wing_rotation_second_order_dynamics;
  bool initialized;
  uint8_t init_loop_count;
  bool airspeed_scheduling;
  bool airspeed_scheduling_nav;
  bool force_rotation_angle;
  bool transition_forward;
  float forward_airspeed;
};

extern struct wing_rotation_controller wing_rotation;

#endif  // WING_ROTATION_CONTROLLER_SERVO_H
