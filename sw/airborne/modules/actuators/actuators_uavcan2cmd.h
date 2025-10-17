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

#ifndef ACTUATORS_UAVCAN2_CMD_H
#define ACTUATORS_UAVCAN2_CMD_H

#include "actuators_uavcan.h"

/** Stub file needed per uavcan interface because of generator */
extern int16_t actuators_uavcan2cmd_values[SERVOS_UAVCAN2CMD_NB];

extern void actuators_uavcan2cmd_set(uint8_t idx, int16_t value);

#if USE_NPS
#define ActuatorUavcan2CmdSet NULL
#define ActuatorsUavcan2CmdInit() {}
#define ActuatorsUavcan2CmdCommit()  {}
#else
#define ActuatorUavcan2CmdSet actuators_uavcan2cmd_set
#define ActuatorsUavcan2CmdInit() actuators_uavcan_init(&uavcan2)
#define ActuatorsUavcan2CmdCommit()  RunOnceEvery(ACTUATORS_UAVCAN_CMD_DIV,actuators_uavcan_cmd_commit(&uavcan2, actuators_uavcan2cmd_values, SERVOS_UAVCAN2CMD_NB))
#endif

#endif /* ACTUATORS_UAVCAN2_CMD_H */
