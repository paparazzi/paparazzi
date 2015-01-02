/*
 * Copyright (C) 2008-2013 The Paparazzi Team
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
 * @file modules/nav/nav_line_osam.h
 *
 */

#ifndef NAV_LINE_OSAM_H
#define NAV_LINE_OSAM_H

#include "std.h"

extern bool_t nav_line_osam_run(uint8_t From_WP, uint8_t To_WP, float radius, float Space_Before, float Space_After);
extern bool_t nav_line_osam_block_run(uint8_t First_WP, uint8_t Last_WP, float radius, float Space_Before,
                                      float Space_After);

#endif
