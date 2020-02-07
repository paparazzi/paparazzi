/*
 * Copyright (C) 2007-2009  ENAC, Pascal Brisset, Antoine Drouin
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
 * @file subsystems/navigation/nav_survey_rectangle.c
 *
 * Automatic survey of a rectangle for fixedwings.
 *
 * Rectangle is defined by two points, sweep can be south-north or west-east.
 */

#ifndef NAV_SURVEY_RECTANGLE_H
#define NAV_SURVEY_RECTANGLE_H

#include "firmwares/fixedwing/nav.h"

typedef enum {NS, WE} survey_orientation_t;

extern void nav_survey_rectangle_init(uint8_t wp1, uint8_t wp2, float grid, survey_orientation_t so);
extern void nav_survey_rectangle(uint8_t wp1, uint8_t wp2);
extern void nav_survey_rectangle_dynamic(uint8_t wp1, uint8_t wp2, float grid);

#define NavSurveyRectangleInit(_wp1, _wp2, _grid, _orientation) nav_survey_rectangle_init(_wp1, _wp2, _grid, _orientation)
#define NavSurveyRectangle(_wp1, _wp2) nav_survey_rectangle(_wp1, _wp2);
#define NavSurveyRectangleDynamic(_wp1, _wp2, _grid) nav_survey_rectangle_dynamic(_wp1, _wp2, _grid)


#endif // NAV_SURVEY_RECTANGLE_H
