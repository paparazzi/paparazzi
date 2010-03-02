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

#include "booz2_analog.h"

#include "std.h"

// battery on AD0.3 on P0.30
// baro    on AD0.1 on P0.28

#define CHAN_BAT  3
#define CHAN_BARO 1


void booz2_analog_init( void ) {

  booz2_analog_init_hw();

}

#ifdef USE_EXTRA_ADC
// Read manually baro (100Hz) and bat (10Hz)
void booz2_analog_periodic( void ) {
  // baro
  RunOnceEvery(5,booz2_analog_baro_read());
  // bat
  RunOnceEvery(50,booz2_analog_bat_read());
}
#endif

