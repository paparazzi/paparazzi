/*
 * Copyright (C) 2014 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/**
 * @file subsystems/gps/gps_piksi.h
 *
 * Driver for Piksi modules from Swift-Nav
 *
 * http://docs.swiftnav.com/wiki/Piksi_Integration_Tutorial
 * https://github.com/swift-nav/sbp_tutorial
 */

#ifndef GPS_PIKSI_H
#define GPS_PIKSI_H

extern bool_t gps_piksi_available;

void gps_piksi_event(void);

/*
 * The GPS event
 */
#define GpsEvent(_sol_available_callback) {         \
    gps_piksi_event();                              \
    if (gps_piksi_available) {                      \
      gps.last_msg_ticks = sys_time.nb_sec_rem;     \
      gps.last_msg_time = sys_time.nb_sec;          \
      if (gps.fix == GPS_FIX_3D) {                  \
        gps.last_3dfix_ticks = sys_time.nb_sec_rem; \
        gps.last_3dfix_time = sys_time.nb_sec;      \
      }                                             \
      _sol_available_callback();                    \
      gps_piksi_available = FALSE;                  \
    }                                               \
  }

#endif /* GPS_PIKSI_H */

