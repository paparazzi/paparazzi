/*
 * $Id$
 *  
 * Copyright (C) 2008  Antoine Drouin
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

/*
 * Simulator hardware implementation of the PNI micromag magnetometer
 */

#include "micromag.h"

#include "6dof.h"

void micromag_hw_init( void ) {}

void micromag_read( void ) {}

void micromag_hw_feed_value(VEC* mag) {
  micromag_values[0] =  mag->ve[AXIS_X];
  micromag_values[1] =  mag->ve[AXIS_Y];
  micromag_values[2] =  mag->ve[AXIS_Z];
  micromag_status = MM_DATA_AVAILABLE;
}

