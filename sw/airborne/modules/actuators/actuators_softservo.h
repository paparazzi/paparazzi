/*
 * Copyright (C) 2023 MAVLab
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

/** @file actuators_soft_servo.h
 *  @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 *  Drive a motor until an ADC port has the correct value
 */

#ifndef ACTUATORS_SOFTSERVO_H
#define ACTUATORS_SOFTSERVO_H

#include "std.h"


/* Main actuator structure */
struct ActuatorsSoftServo {
  int32_t cmds;
  int32_t servo_pprz_cmd;
  uint16_t adc_wing_rotation;
  float wing_rotation_speed;
  float wing_rotation_first_order_dynamics;
  float wing_rotation_second_order_dynamics;
  bool initialized;
  uint8_t init_loop_count;
};

/* Functions used in actuator macros */
extern struct ActuatorsSoftServo actuators_softservo;
extern void actuators_softservo_init(void);
extern void actuators_softservo_update(void);

/* Actuator macros */
#define ActuatorSoftServoSet(_i, _v) { actuators_softservo.cmds = _v; }
#define ActuatorsSoftServoInit() actuators_softservo_init()
#define ActuatorsSoftServoCommit() actuators_softservo_update()


#endif /* ACTUATORS_SOFTSERVO_H */
