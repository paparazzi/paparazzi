/*
 * Copyright (C) 2012 Freek van Tienen
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
 * @file gps_sirf.h
 * @brief Sirf protocol specific code
 *
 */

#ifndef GPS_SIRF_H
#define GPS_SIRF_H

#include "std.h"

#define GPS_NB_CHANNELS 16
#define SIRF_MAXLEN 255

//Read states
#define UNINIT  0
#define GOT_A0  1
#define GOT_A2  2
#define GOT_B0  3

struct GpsSirf {
  bool_t msg_available;
  bool_t pos_available;
  char msg_buf[SIRF_MAXLEN];  ///< buffer for storing one nmea-line
  int msg_len;
  int read_state;
};

extern struct GpsSirf gps_sirf;

//Invert bytes
#define Invert2Bytes(x) ((x>>8) | (x<<8))
#define Invert4Bytes(x) ((x>>24) | ((x<<8) & 0x00FF0000) | ((x>>8) & 0x0000FF00) | (x<<24))

/** Message ID 2 from GPS. Total payload length should be 41 bytes. */
struct sirf_msg_2 {
  uint8_t msg_id;     ///< hex value 0x02 ( = decimal 2)
  int32_t x_pos;    ///< x-position in m
  int32_t y_pos;    ///< y-position in m
  int32_t z_pos;    ///< z-position in m
  int16_t vx;       ///< x-velocity * 8 in m/s
  int16_t vy;       ///< y-velocity * 8 in m/s
  int16_t vz;       ///< z-velocity * 8 in m/s
  uint8_t mode1;
  uint8_t hdop;     ///< horizontal dilution of precision *5 (0.2 precision)
  uint8_t mode2;
  uint16_t week;
  uint32_t tow;       ///< time of week in seconds * 10^2
  uint8_t num_sat;    ///< Number of satellites in fix
  uint8_t ch1prn;     ///< pseudo-random noise, 12 channels
  uint8_t ch2prn;
  uint8_t ch3prn;
  uint8_t ch4prn;
  uint8_t ch5prn;
  uint8_t ch6prn;
  uint8_t ch7prn;
  uint8_t ch8prn;
  uint8_t ch9prn;
  uint8_t ch10prn;
  uint8_t ch11prn;
  uint8_t ch12prn;
} __attribute__((packed));


/** Message ID 41 from GPS. Total payload length should be 91 bytes. */
struct sirf_msg_41 {
  uint8_t msg_id;       ///< hex value 0x29 (= decimal 41)
  uint16_t nav_valid;     ///< if equal to 0x0000, then navigation solution is valid
  uint16_t nav_type;
  uint16_t extended_week_number;
  uint32_t tow;       ///< time of week  in seconds *10^3]
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint16_t second;
  uint32_t sat_id;      ///< satellites used in solution. Each satellite corresponds with a bit, e.g. bit 1 ON = SV 1 is used in solution
  int32_t latitude;     ///< in degrees (+= North) *10^7
  int32_t longitude;    ///< in degrees (+= East) *10*7
  int32_t alt_ellipsoid;  ///< in meters *10^2
  int32_t alt_msl;      ///< in meters *10^2
  int8_t map_datum;
  uint16_t sog;       ///< speed over ground, in m/s * 10^2
  uint16_t cog;       ///< course over ground, in degrees clockwise from true north * 10^2
  int16_t mag_var;      ///< not implemented
  int16_t climb_rate;   ///< in m/s * 10^2
  int16_t heading_rate;   ///< in deg/s * 10^2
  uint32_t ehpe;      ///< estimated horizontal position error, in meters * 10^2
  uint32_t evpe;      ///< estimated vertical position error, in meters * 10^2
  uint32_t ete;       ///< estimated time error, in seconds * 10^2
  uint16_t ehve;      ///< estimated horizontal velocity error in m/s * 10^2
  int32_t clock_bias;     ///< in m * 10^2
  uint32_t clock_bias_err;  ///< in m * 10^2
  int32_t clock_drift;    ///< in m/s * 10^2
  uint32_t clock_drift_err; ///< in m/s * 10^2
  uint32_t distance;    ///< Distance traveled since reset in m
  uint16_t distance_err;  ///< in meters
  uint16_t heading_err;   ///< in degrees * 10^2
  uint8_t num_sat;      ///< Number of satellites used for solution
  uint8_t hdop;       ///< Horizontal dilution of precision x 5 (0.2 precision)
  uint8_t add_info;     ///< Additional mode info
} __attribute__((packed));

/*
 * This part is used by the autopilot to read data from a uart
 */
#include "mcu_periph/link_device.h"

extern void sirf_parse_char(uint8_t c);
extern void sirf_parse_msg(void);
extern void gps_sirf_msg(void (* _cb)(void));

static inline void GpsEvent(void (* _sol_available_callback)(void))
{
  struct link_device *dev = &((GPS_LINK).device);

  if (dev->char_available(dev->periph)) {
    while (dev->char_available(dev->periph)) {
      sirf_parse_char(dev->get_byte(dev->periph));
    }
  }
  if (gps_sirf.msg_available) {
    gps_sirf_msg(_sol_available_callback);
  }
}

#endif /* GPS_SIRF_H */
