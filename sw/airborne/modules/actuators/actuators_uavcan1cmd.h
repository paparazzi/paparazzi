/*
 * Copyright (C) 2023 Freek van Tienen <freek.v.tienen@gmail.com>
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

#ifndef ACTUATORS_UAVCAN1_CMD_H
#define ACTUATORS_UAVCAN1_CMD_H

#include "actuators_uavcan.h"

/** Stub file needed per uavcan interface because of generator */
extern int16_t actuators_uavcan1cmd_values[SERVOS_UAVCAN1CMD_NB];

#if USE_NPS
#define ActuatorsUavcan1CmdInit() {}
#define ActuatorUavcan1CmdSet(_i, _v) {}
#define ActuatorsUavcan1CmdCommit()  {}
#else
#define ActuatorsUavcan1CmdInit() actuators_uavcan_init(&uavcan1)
#define ActuatorUavcan1CmdSet(_i, _v) { actuators_uavcan1cmd_values[_i] = _v; }
#define ActuatorsUavcan1CmdCommit()  RunOnceEvery(ACTUATORS_UAVCAN_CMD_DIV,actuators_uavcan_cmd_commit(&uavcan1, actuators_uavcan1cmd_values, SERVOS_UAVCAN1CMD_NB))
#endif

#endif /* ACTUATORS_UAVCAN1_CMD_H */