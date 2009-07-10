/*
 * Copyright (C) 2008 Joby Energy
 *  
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

/** \file mercury_ap.h
 */

#ifndef __MERCURY_AP_H__
#define __MERCURY_AP_H__

#include "airframe.h"

extern pprz_t mixed_commands[PROPS_NB];

void csc_ap_init( void );
void csc_ap_periodic (uint8_t _in_flight, uint8_t kill);


#endif 
