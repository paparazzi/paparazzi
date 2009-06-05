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

/** \file csc_ap.h
 */

#ifndef __CSC_AP_H__
#define __CSC_AP_H__

#include "types.h"
#include "std.h"
#include "pprz_algebra_float.h"

struct control_gains {
  float pitch_kp;
  float roll_kp;
  float yaw_kp;
  float pitch_kd;
  float roll_kd;
  float yaw_kd;
};

struct control_reference {
  struct FloatEulers eulers;
  struct FloatRates rates;
};

struct control_trims {
  int elevator;
  int aileron;
  int rudder;
};

extern struct control_gains csc_gains;
extern struct control_reference csc_reference;
extern struct control_trims csc_trims;

void csc_ap_init( void );
void csc_ap_periodic (void );


#endif 
