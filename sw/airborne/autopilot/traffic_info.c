/*
 * $Id$
 *  
 * Copyright (C) 2005  Pascal Brisset, Antoine Drouin
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
/** \file nav.c
 *  \brief Informations relative to the other aircrafts
 *
 */

#include <inttypes.h>
#include "flight_plan.h"

#define NB_OTHERS 4

struct ac_info_ {float east, north, heading, alt;};

struct ac_info_ the_others[NB_OTHERS];

void
set_the_other(uint8_t id, float utm_x, float utm_y, float heading, float alt) {
  if (id < NB_OTHERS) {
    the_others[id].east = utm_x -  NAV_UTM_EAST0;
    the_others[id].north = utm_y - NAV_UTM_NORTH0;
    the_others[id].heading = heading;
    the_others[id].alt = alt;
  }
}

struct ac_info_ * get_the_other(uint8_t id) {
  id = (id < NB_OTHERS ? id : NB_OTHERS - 1);
  return &the_others[id];
}
