/*
 * Copyright (C) 2021 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifndef ACTUATORS_DRONECAN2_H
#define ACTUATORS_DRONECAN2_H

#include "actuators_dronecan.h"

/** Stub file needed per interface because of generator */
extern int16_t actuators_dronecan2_values[SERVOS_DRONECAN2_NB];

#if USE_NPS
#define ActuatorsDronecan2Init() {}
#define ActuatorDronecan2Set(_i, _v) {}
#define ActuatorsDronecan2Commit()  {}
#else
#define ActuatorsDronecan2Init() actuators_dronecan_init(&dronecan2)
#define ActuatorDronecan2Set(_i, _v) { actuators_dronecan2_values[_i] = _v; }
#define ActuatorsDronecan2Commit()  actuators_dronecan_commit(&dronecan2, actuators_dronecan2_values, SERVOS_DRONECAN2_NB)
#endif

#endif /* ACTUATORS_DRONECAN2_H */