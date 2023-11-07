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

// Paramaters
struct wing_rotation_controller_t {
  float wing_angle_deg;                       ///< Wing angle in degrees
  float wing_angle_deg_sp;                    ///< Wing angle setpoint in degrees

  int32_t servo_pprz_cmd;                     ///< Servo command in pprz
  uint16_t adc_wing_rotation;                 ///< ADC value of wing

  float wing_rotation_speed;                  ///< Rate limiter state variable 1
  float wing_angle_virtual_deg_sp;            ///< Rate limiter state variable 2
  float wing_rotation_first_order_dynamics;   ///< Rate limiter for wing rotation
  float wing_rotation_second_order_dynamics;  ///< Acceleration limiter for wing rotation

  bool initialized;                           ///< Wing rotation controller initialized                
  uint8_t init_loop_count;                    ///< Wing rotation controller initialization loop count
};

extern struct wing_rotation_controller_t wing_rotation_controller;

#endif  // WING_ROTATION_CONTROLLER_SERVO_H
