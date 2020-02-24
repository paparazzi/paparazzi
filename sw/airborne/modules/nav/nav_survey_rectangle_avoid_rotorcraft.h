/*
 * Copyright (C) 2007-2009  ENAC, Pascal Brisset, Antoine Drouin
 *                    2015  NAC-VA, Eduardo Lavratti
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/nav/nav_survey_rectangle_rotorcraft.h
 *
 * Automatic survey of a rectangle for rotorcraft.
 *
 * Rectangle is defined by two points, sweep can be south-north or west-east.
 */

#ifndef NAV_SURVEY_RECTANGLE_AVOID_ROTORCRAFT_H
#define NAV_SURVEY_RECTANGLE_AVOID_ROTORCRAFT_H

#include "firmwares/rotorcraft/navigation.h"
#include "modules/nav/nav_survey_rectangle_rotorcraft.h"

extern float sx1, sx2, sy1, sy2, ax1, ax2, ay1, ay2;
extern uint8_t wp1, wp2, awp1, awp2, util_wp1, util_wp2;
extern int type, setup, survey_num, max_num_surveys;

extern void next_partial_survey();
extern void setup_survey(float x1, float y1, float x2, float y2);
extern void nav_survey_rectangle_avoid_rotorcraft_init();
extern void nav_survey_rectangle_avoid_rotorcraft_setup(uint8_t wp1, uint8_t wp2, uint8_t wp3, uint8_t wp4, uint8_t wp5, uint8_t wp6, float grid, survey_orientation_t so);
extern bool nav_survey_rectangle_avoid_rotorcraft_run();

#endif // NAV_SURVEY_RECTANGLE_AVOID_ROTORCRAFT_H
