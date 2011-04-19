/*
 * $Id$
 *
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

#include "ms2100.h"

volatile uint8_t ms2100_status;
volatile int16_t ms2100_values[MS2100_NB_AXIS];

void ms2100_init( void ) {

  ms2100_arch_init();

  uint8_t i;
  for (i=0; i<MS2100_NB_AXIS; i++)
    ms2100_values[i] = 0;
  ms2100_status = MS2100_IDLE;
}

void ms2100_reset() {
  ms2100_status = MS2100_IDLE;
}
