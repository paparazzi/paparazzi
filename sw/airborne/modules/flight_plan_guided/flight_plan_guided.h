/*
 * Copyright (C) 2016 - IMAV 2016
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
/**
 * @file modules/computer_vision/marker_tracking.h
 * @author IMAV 2016
 */

#ifndef FLIGHT_PLAN_GUIDED_PLUGIN_H
#define FLIGHT_PLAN_GUIDED_PLUGIN_H

#include <stdint.h>

// Module functions
void flight_plan_guided_init(void);

// Flight Plan functions
extern uint8_t KillEngines(void);
extern uint8_t StartEngines(void);
extern uint8_t reset_alt(void);
extern uint8_t hover(float height_above_target, float vz_bottom_ref);
extern uint8_t MoveForward(float vx);

#endif