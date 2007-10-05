/*
 * Paparazzi $Id$
 *  
 * Copyright (C) 2005 Pascal Brisset, Antoine Drouin
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
/** \file main.c
 * \brief main loop used both on single and dual MCU configuration */


#include "sys_time.h"

#ifdef FBW
#include "main_fbw.h"
#define Fbw(f) f ## _fbw()
#else
#define Fbw(f)
#endif

#ifdef AP
#include "main_ap.h"
#define Ap(f) f ## _ap()
#else
#define Ap(f)
#endif

int main( void ) {
  Fbw(init);
  Ap(init);
  InitSysTimePeriodic()
  while (1) {
    if (sys_time_periodic()) {
      Fbw(periodic_task);
      Ap(periodic_task);
    }
    Fbw(event_task);
    Ap(event_task);
  }
  return 0;
}
