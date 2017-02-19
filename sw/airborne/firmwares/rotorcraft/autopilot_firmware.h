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
 * @file firmwares/rotorcraft/autopilot_firmware.h
 *
 * Rotorcraft specific autopilot interface
 * and initialization
 */

#ifndef AUTOPILOT_FIRMWARE_H
#define AUTOPILOT_FIRMWARE_H

#include "std.h"
#include "autopilot.h"

extern uint8_t autopilot_mode_auto2;

extern void autopilot_firmware_init(void);

#endif /* AUTOPILOT_FIRMWARE_H */
