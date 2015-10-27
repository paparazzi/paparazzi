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
  bool_t  msg_available;
  uint8_t msg_id;

  uint8_t status;
  uint16_t len;
  uint8_t msg_idx;
  uint8_t checksum;
  uint8_t error_cnt;
  enum GpsSkytraqError error_last;

  struct LtpDef_i ref_ltp;
};

extern struct GpsSkytraq gps_skytraq;


/*
 * This part is used by the autopilot to read data from a uart
 */
#include "mcu_periph/link_device.h"

extern void gps_skytraq_read_message(void);
extern void gps_skytraq_parse(uint8_t c);
extern void gps_skytraq_msg(void);

static inline void GpsEvent(void)
{
  struct link_device *dev = &((GPS_LINK).device);

  while (dev->char_available(dev->periph)) {
    gps_skytraq_parse(dev->get_byte(dev->periph));
    if (gps_skytraq.msg_available) {
      gps_skytraq_msg();
    }
  }
}

#endif /* GPS_SKYTRAQ_H */
