/*
 * Copyright (C) 2023 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file "modules/checks/pfc_actuators.h"
 * @author Freek van Tienen <freek.v.tienen@gmail.com>
 * Checks the actuators with feedback before takeoff
 */

#ifndef PFC_ACTUATORS_H
#define PFC_ACTUATORS_H

#include "std.h"

extern void pfc_actuators_init(void);
extern void pfc_actuators_run(void);
extern void pfc_actuators_start(bool start);
extern int16_t pfc_actuators_value(uint8_t idx, int16_t value);

#endif /* PFC_ACTUATORS_H */
