/*
 * Copyright (C) 2018 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file firmwares/rover/autopilot_generated.h
 *
 * Autopilot generated implementation
 * Calls the code generated from autopilot XML file
 *
 */

#ifndef AUTOPILOT_GENERATED_H
#define AUTOPILOT_GENERATED_H

#include "std.h"
#include "generated/autopilot_core_ap.h"
#include "generated/airframe.h"
#include "state.h"

extern void autopilot_generated_init(void);
extern void autopilot_generated_periodic(void);
extern void autopilot_generated_on_rc_frame(void);
extern void autopilot_generated_set_mode(uint8_t new_autopilot_mode);
extern void autopilot_generated_SetModeHandler(float new_autopilot_mode); // handler for dl_setting
extern void autopilot_generated_set_motors_on(bool motors_on);

#endif /* AUTOPILOT_GENERATED_H */

