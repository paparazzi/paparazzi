/*
 * Copyright (C) 2022 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file firmwares/fixedwing/main_recovery.h
 *
 * Recovery mode: run manual mode in case of hardfault
 * Based on legacy FBW
 *
 */
#ifndef MAIN_RECOVERY_H
#define MAIN_RECOVERY_H

#include "std.h"

/** recovery modes */
#define RECOVERY_MODE_MANUAL   0
#define RECOVERY_MODE_FAILSAFE 2 // for compatibility will old FBW modes

extern uint8_t recovery_mode;

void main_recovery_init(void);
void main_recovery_periodic(void);
void main_recovery_event(void);

#endif /* MAIN_RECOVERY_H */
