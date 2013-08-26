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
 * @file modules/nav/nav_smooth.h
 *
 * Smooth navigation to wp_a along an arc (around wp_cd),
 * a segment (from wp_rd to wp_ta) and a second arc (around wp_ca).
 */

#ifndef SNAV_H
#define SNAV_H

#include "std.h"


extern float snav_desired_tow; /* time of week, s */

bool_t snav_init(uint8_t wp_a, float desired_course_rad, float radius);
bool_t snav_circle1(void);
bool_t snav_route(void);
bool_t snav_circle2(void);
bool_t snav_on_time(float radius);

#endif // SNAV_H
