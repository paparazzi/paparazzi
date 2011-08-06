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

#define SKYTRAQ_SYNC1 0xA0
#define SKYTRAQ_SYNC2 0xA1

#define SKYTRAQ_SYNC3 0x0D
#define SKYTRAQ_SYNC4 0x0A


#define SKYTRAQ_ID_NAVIGATION_DATA 0XA8

#define SKYTRAQ_NAVIGATION_DATA_FixMode(_payload) (uint8_t) (*((uint8_t*)_payload+2-2))
#define SKYTRAQ_NAVIGATION_DATA_NumSV(_payload)   (uint8_t) (*((uint8_t*)_payload+3-2))

//#define SKYTRAQ_NAVIGATION_DATA_TOW(_payload)     (uint32_t)(_payload[7] + (((uint32_t)_payload[6])<<8) + (((uint32_t)_payload[5])<<16) + (((uint32_t)_payload[4])<<24))
#define SKYTRAQ_NAVIGATION_DATA_TOW(_payload)     __builtin_bswap32(*(uint32_t*)&_payload[ 6-2])
#define SKYTRAQ_NAVIGATION_DATA_LAT(_payload)     __builtin_bswap32(*( int32_t*)&_payload[10-2])
#define SKYTRAQ_NAVIGATION_DATA_LON(_payload)     __builtin_bswap32(*( int32_t*)&_payload[14-2])
#define SKYTRAQ_NAVIGATION_DATA_AEL(_payload)     __builtin_bswap32(*(uint32_t*)&_payload[18-2])
#define SKYTRAQ_NAVIGATION_DATA_ASL(_payload)     __builtin_bswap32(*(uint32_t*)&_payload[22-2])
//#define SKYTRAQ_NAVIGATION_DATA_GDOP(_payload)    __builtin_bswap16(*(uint16_t*)&_payload[26-2])
//#define SKYTRAQ_NAVIGATION_DATA_PDOP(_payload)    __builtin_bswap(*(uint16_t*)&_payload[28-2])

#define SKYTRAQ_NAVIGATION_DATA_ECEFX(_payload)   __builtin_bswap32(*( int32_t*)&_payload[36-2])
#define SKYTRAQ_NAVIGATION_DATA_ECEFY(_payload)   __builtin_bswap32(*( int32_t*)&_payload[40-2])
#define SKYTRAQ_NAVIGATION_DATA_ECEFZ(_payload)   __builtin_bswap32(*( int32_t*)&_payload[44-2])
#define SKYTRAQ_NAVIGATION_DATA_ECEFVX(_payload)  __builtin_bswap32(*( int32_t*)&_payload[48-2])
#define SKYTRAQ_NAVIGATION_DATA_ECEFVY(_payload)  __builtin_bswap32(*( int32_t*)&_payload[52-2])
#define SKYTRAQ_NAVIGATION_DATA_ECEFVZ(_payload)  __builtin_bswap32(*( int32_t*)&_payload[56-2])



/* last error type */
#define GPS_SKYTRAQ_ERR_NONE         0
#define GPS_SKYTRAQ_ERR_OVERRUN      1
#define GPS_SKYTRAQ_ERR_MSG_TOO_LONG 2
#define GPS_SKYTRAQ_ERR_CHECKSUM     3
#define GPS_SKYTRAQ_ERR_OUT_OF_SYNC  4
#define GPS_SKYTRAQ_ERR_UNEXPECTED   5

#define GPS_SKYTRAQ_MAX_PAYLOAD 255
struct GpsSkytraq {
  uint8_t msg_buf[GPS_SKYTRAQ_MAX_PAYLOAD] __attribute__ ((aligned));
  bool_t  msg_available;
  uint8_t msg_id;

  uint8_t  status;
  uint16_t len;
  uint8_t  msg_idx;
  uint8_t  checksum;
  uint8_t  error_cnt;
  uint8_t  error_last;
};

extern struct GpsSkytraq gps_skytraq;


/*
 * This part is used by the autopilot to read data from a uart
 */
#define __GpsLink(dev, _x) dev##_x
#define _GpsLink(dev, _x)  __GpsLink(dev, _x)
#define GpsLink(_x) _GpsLink(GPS_LINK, _x)

#define GpsBuffer() GpsLink(ChAvailable())

#define GpsEvent(_sol_available_callback) {                     \
    if (GpsBuffer()) {                                          \
      ReadGpsBuffer();                                          \
    }                                                           \
    if (gps_skytraq.msg_available) {                            \
      gps_skytraq_read_message();                               \
      if (gps_skytraq.msg_id == SKYTRAQ_ID_NAVIGATION_DATA) {	\
        if (gps.fix == GPS_FIX_3D)                              \
          gps.last_fix_ticks = cpu_time_ticks;                  \
          gps.last_fix_time = cpu_time_sec;                     \
        _sol_available_callback();                              \
      }                                                         \
      gps_skytraq.msg_available = FALSE;                        \
    }                                                           \
  }

#define ReadGpsBuffer() {						\
    while (GpsLink(ChAvailable())&&!gps_skytraq.msg_available)	\
      gps_skytraq_parse(GpsLink(Getch()));				\
  }


extern void gps_skytraq_read_message(void);
extern void gps_skytraq_parse(uint8_t c);

#endif /* GPS_SKYTRAQ_H */
