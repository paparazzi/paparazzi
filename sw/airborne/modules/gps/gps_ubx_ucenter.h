/*
 * Copyright (C) 2008-2011 The Paparazzi Team
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
 * @file modules/gps/gps_ubx_ucenter.h
 * @brief Configure Ublox GPS
 *
 */

#ifndef GPS_UBX_UCENTER_H
#define GPS_UBX_UCENTER_H

#include "std.h"

/** U-Center Variables */
#define GPS_UBX_UCENTER_CONFIG_STEPS	  18

struct gps_ubx_ucenter_struct
{
  uint8_t status;
  uint8_t reply;
  uint8_t cnt;

  uint32_t baud_init;
  uint32_t baud_run;

  uint8_t sw_ver_h;
  uint8_t sw_ver_l;

  uint16_t hw_ver_h;
  uint16_t hw_ver_l;

  char replies[GPS_UBX_UCENTER_CONFIG_STEPS];
};

extern struct gps_ubx_ucenter_struct gps_ubx_ucenter;

extern void gps_ubx_ucenter_init(void);
extern void gps_ubx_ucenter_periodic(void);
extern void gps_ubx_ucenter_event(void);

#endif


