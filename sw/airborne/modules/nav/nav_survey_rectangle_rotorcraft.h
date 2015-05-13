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

#ifndef NAV_SURVEY_RECTANGLE_ROTORCRAFT_H
#define NAV_SURVEY_RECTANGLE_ROTORCRAFT_H

#include "firmwares/rotorcraft/navigation.h"

typedef enum {NS, WE} survey_orientation_t;

extern float sweep;
extern uint16_t rectangle_survey_sweep_num;
extern bool_t interleave;


extern void nav_survey_rectangle_rotorcraft_init(void);
extern bool_t nav_survey_rectangle_rotorcraft_setup(uint8_t wp1, uint8_t wp2, float grid1, survey_orientation_t so);
extern bool_t nav_survey_rectangle_rotorcraft_run(uint8_t wp1, uint8_t wp2);

#define NavSurveyRectangleInit(_wp1, _wp2, _grid, _orientation) nav_survey_rectangle_rotorcraft_setup(_wp1, _wp2, _grid, _orientation)
#define NavSurveyRectangle(_wp1, _wp2) nav_survey_rectangle_rotorcraft_run(_wp1, _wp2)

#endif // NAV_SURVEY_RECTANGLE_ROTORCRAFT_H
