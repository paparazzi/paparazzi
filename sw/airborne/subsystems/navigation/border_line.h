/*
 * $Id$
 *
 * Copyright (C) 2012, Tobias Muench
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
 *
 */

/**
 * @file subsystems/navigation/border_line.h
 * @brief navigate along a border line (line 1-2) with turns in the same direction
 */

#ifndef BORDER_LINE_H
#define BORDER_LINE_H

#include "std.h"

extern bool_t border_line_init( void );
extern bool_t border_line(uint8_t wp1, uint8_t wp2, float radius);

#endif

/* BORDER_LINE_H */
