/*
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file firmwares/fixedwing/autopilot_firmware.h
 *
 * Fixedwing specific autopilot interface
 * and initialization
 */

#ifndef AUTOPILOT_FIRMWARE_H
#define AUTOPILOT_FIRMWARE_H

#include "std.h"
#include "autopilot.h"
#include "subsystems/electrical.h"

// FIXME, move to control
#define LATERAL_MODE_MANUAL    0
#define LATERAL_MODE_ROLL_RATE 1
#define LATERAL_MODE_ROLL      2
#define LATERAL_MODE_COURSE    3
#define LATERAL_MODE_NB        4
extern uint8_t lateral_mode;

// ap copy of fbw readings
extern struct Electrical ap_electrical;

/** Second MCU status (FBW part)
 */
extern uint8_t  mcu1_status;

/** Init function
 */
extern void autopilot_firmware_init(void);

#endif /* AUTOPILOT_FIRMWARE_H */


