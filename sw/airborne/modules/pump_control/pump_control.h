/*
 * Copyright (C) 2026 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file "modules/pump_control/pump_control.h"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Pump control for IMAV2026
 */

#ifndef PUMP_CONTROL_H
#define PUMP_CONTROL_H

#include "std.h"

extern void pump_control_init(void);
extern void pump_activate(void);
extern void pump_purge(void);
extern void pump_stop(void);
extern bool pump_done(float duration, float timeout);
extern bool pump_pumping(void);
void pump_control_report(void);

void pump_control_handler(float value);
extern float pump_ctrl_state;

#endif  // PUMP_CONTROL_H
