/*
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
 */

/**
 * @file subsystems/navigation/traffic_info.h
 *
 * Information relative to the other aircrafts.
 *
 */

#ifndef TI_H
#define TI_H

#define NB_ACS_ID 256
#define NB_ACS 24

struct ac_info_ {
  uint8_t ac_id;
  float east; /* m relative to nav_utm_east0 */
  float north; /* m relative to nav_utm_north0 */
  float course; /* rad (CW) */
  float alt; /* m */
  float gspeed; /* m/s */
  float climb; /* m/s */
  uint32_t itow; /* ms */
};

extern uint8_t acs_idx;
extern uint8_t the_acs_id[NB_ACS_ID];
extern struct ac_info_ the_acs[NB_ACS];

// 0 is reserved for ground station (id=0)
// 1 is reserved for this AC (id=AC_ID)
#define SetAcInfo(_id, _utm_x /*m*/, _utm_y /*m*/, _course/*rad(CW)*/, _alt/*m*/,_gspeed/*m/s*/,_climb, _itow) { \
    if (acs_idx < NB_ACS) {                                             \
      if (_id > 0 && the_acs_id[_id] == 0) {                            \
        the_acs_id[_id] = acs_idx++;                                    \
        the_acs[the_acs_id[_id]].ac_id = (uint8_t)_id;                  \
      }                                                                 \
      the_acs[the_acs_id[_id]].east = _utm_x -  nav_utm_east0;          \
      the_acs[the_acs_id[_id]].north = _utm_y - nav_utm_north0;         \
      the_acs[the_acs_id[_id]].course = _course;                        \
      the_acs[the_acs_id[_id]].alt = _alt;                              \
      the_acs[the_acs_id[_id]].gspeed = _gspeed;                        \
      the_acs[the_acs_id[_id]].climb = _climb;                          \
      the_acs[the_acs_id[_id]].itow = (uint32_t)_itow;                  \
    }                                                                   \
  }

extern void traffic_info_init(void);

struct ac_info_ *get_ac_info(uint8_t id);

#endif
