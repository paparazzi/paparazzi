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

#ifndef ACTUATORS_UAVCAN2_H
#define ACTUATORS_UAVCAN2_H

#include "actuators_uavcan.h"

/** Stub file needed per interface because of generator */
extern int16_t actuators_uavcan2_values[SERVOS_UAVCAN2_NB];

#if USE_NPS
#define ActuatorsUavcan2Init() {}
#define ActuatorUavcan2Set(_i, _v) {}
#define ActuatorsUavcan2Commit()  {}
#else
#define ActuatorsUavcan2Init() actuators_uavcan_init(&uavcan2)
#define ActuatorUavcan2Set(_i, _v) { actuators_uavcan2_values[_i] = _v; }
#define ActuatorsUavcan2Commit()  actuators_uavcan_commit(&uavcan2, actuators_uavcan2_values, SERVOS_UAVCAN2_NB)
#endif

#endif /* ACTUATORS_UAVCAN2_H */