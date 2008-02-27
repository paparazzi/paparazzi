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
/** \file traffic_info.h
 *  \brief Informations relative to the other aircrafts
 *
 */

#ifndef TI_H
#define TI_H

#define NB_ACS 24

struct ac_info_ {
  float east; /* m */
  float north; /* m */
  float course; /* rad (CW) */
  float alt; /* m */
  float gspeed; /* m/s */
};

extern struct ac_info_ the_acs[NB_ACS];

#define SetAcInfo(_id, _utm_x /*m*/, _utm_y /*m*/, _course/*rad(CW)*/, _alt/*m*/,_gspeed/*m/s*/) { \
  if (_id < NB_ACS) { \
    the_acs[_id].east = _utm_x -  nav_utm_east0; \
    the_acs[_id].north = _utm_y - nav_utm_north0; \
    the_acs[_id].course = _course; \
    the_acs[_id].alt = _alt; \
    the_acs[_id].gspeed = _gspeed; \
  } \
}


struct ac_info_ *
get_ac_info(uint8_t id);

#endif
