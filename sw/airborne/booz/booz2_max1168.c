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

#include "booz2_max1168.h"


volatile uint8_t booz2_max1168_status;
uint16_t booz2_max1168_values[MAX1168_NB_CHAN];

uint8_t do_booz2_max1168_read;

extern void booz2_max1168_init( void ) {

  booz2_max1168_hw_init();

  do_booz2_max1168_read = false;

  uint8_t i;
  for (i=0; i<MAX1168_NB_CHAN; i++)
    booz2_max1168_values[i] = 0;

  booz2_max1168_status = STA_MAX1168_IDLE;
}
