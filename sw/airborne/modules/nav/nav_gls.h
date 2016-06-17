/*
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
 */

/**
 * @file modules/nav/nav_gls.h
 * @brief gps landing system
 */

#ifndef NAV_GLS_H
#define NAV_GLS_H

#include "std.h"
#include "paparazzi.h"

extern bool gls_start(uint8_t _af, uint8_t _sd, uint8_t _tod, uint8_t _td);
extern bool gls_run(uint8_t _af, uint8_t _sd, uint8_t _tod, uint8_t _td);

#endif // NAV_GLS_H
