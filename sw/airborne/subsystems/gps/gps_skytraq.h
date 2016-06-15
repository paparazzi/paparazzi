/*
 * Copyright (C) 2010 Antoine Drouin <poinix@gmail.com>
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

#ifndef GPS_SKYTRAQ_H
#define GPS_SKYTRAQ_H

#include "mcu_periph/uart.h"

#define SKYTRAQ_ID_NAVIGATION_DATA 0XA8

#ifndef PRIMARY_GPS
#define PRIMARY_GPS GPS_SKYTRAQ
#endif

/* last error type */
enum GpsSkytraqError {
  GPS_SKYTRAQ_ERR_NONE = 0,
  GPS_SKYTRAQ_ERR_OVERRUN,
  GPS_SKYTRAQ_ERR_MSG_TOO_LONG,
  GPS_SKYTRAQ_ERR_CHECKSUM,
  GPS_SKYTRAQ_ERR_OUT_OF_SYNC,
  GPS_SKYTRAQ_ERR_UNEXPECTED
};

#define GPS_SKYTRAQ_MAX_PAYLOAD 255
struct GpsSkytraq {
  uint8_t msg_buf[GPS_SKYTRAQ_MAX_PAYLOAD];
  bool  msg_available;
  uint8_t msg_id;

  uint8_t status;
  uint16_t len;
  uint8_t msg_idx;
  uint8_t checksum;
  uint8_t error_cnt;
  enum GpsSkytraqError error_last;

  struct LtpDef_i ref_ltp;

  struct GpsState state;
};

extern struct GpsSkytraq gps_skytraq;

extern void gps_skytraq_init(void);
extern void gps_skytraq_event(void);

#define gps_skytraq_periodic_check() gps_periodic_check(&gps_skytraq.state)

#endif /* GPS_SKYTRAQ_H */
