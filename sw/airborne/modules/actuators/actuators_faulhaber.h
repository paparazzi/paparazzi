/*
 * Copyright (C) Freek van Tienen <freek.v.tienen@gmail.com>
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

/**
 * @file "modules/actuators/actuators_faulhaber.h"
 * @author Freek van Tienen
 * Serial Connection module between ap and a faulhaber motor controller
 */

#ifndef ACTUATORS_FAULHABER_H
#define ACTUATORS_FAULHABER_H

#include "std.h"

enum faulhaber_modes_t {
  FH_MODE_INIT,
  FH_MODE_VELOCITY,
  FH_MODE_ERROR,
  FH_MODE_REQ_ERR,
  FH_MODE_RESET_ERR,
};

struct faulhaber_t {
  enum faulhaber_modes_t mode;    ///< Current mode of the controller
  uint8_t state;                  ///< The state of the mode
  
  float p_gain;                   ///< The proportional gain of the velocity controller
  int32_t max_velocity;           ///< The maximum velocity of the controller

  int32_t setpoint_position;      ///< The setpoint position controlled from the actuator
  int32_t real_position;          ///< The real position from the feedback of the controller
  int32_t target_velocity;        ///< The target velocity send to the controller

  bool homing_completed;      ///< Once the homing is completed
  bool position_ready;        ///< Ready for receiving (new) positions
  bool target_reached;        ///< When the target position is reached
};
extern struct faulhaber_t faulhaber;

extern void actuators_faulhaber_init(void);
extern void actuators_faulhaber_periodic(void);
extern void actuators_faulhaber_event(void);
extern void actuators_faulhaber_SetMode(uint8_t mode);


#define ACTUATORS_FAULHABER_COMMAND_SCALE 1000.0f

#if USE_NPS
#define ActuatorsFaulhaberInit() {}
#define ActuatorFaulhaberSet(_i, _v) {}
#define ActuatorsFaulhaberCommit()  {}
#else
#define ActuatorsFaulhaberInit() actuators_faulhaber_init()
#define ActuatorFaulhaberSet(_i, _v) { faulhaber.setpoint_position = ((get_servo_max_FAULHABER(0)-_v) + get_servo_min_FAULHABER(0))*ACTUATORS_FAULHABER_COMMAND_SCALE; }
#define ActuatorsFaulhaberCommit()  {}
#endif

#endif /* ACTUATORS_FAULHABER_H */
