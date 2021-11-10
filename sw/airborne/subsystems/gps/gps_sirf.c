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


#include "subsystems/gps.h"
#include "modules/core/abi.h"
#include "led.h"

#include "math/pprz_geodetic_float.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include "gps_sirf.h"

#include "pprzlink/pprzlink_device.h"
#include "mcu_periph/uart.h"

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


struct GpsSirf gps_sirf;

void sirf_parse_char(uint8_t c);
void sirf_parse_msg(void);
void gps_sirf_msg(void);

void sirf_parse_2(void);
void sirf_parse_41(void);

void gps_sirf_init(void)
{
  gps_sirf.msg_available = false;
  gps_sirf.msg_valid = false;
  gps_sirf.msg_len = 0;
  gps_sirf.read_state = 0;
}

void gps_sirf_msg(void)
{
  // current timestamp
  uint32_t now_ts = get_sys_time_usec();
  gps_sirf.state.last_msg_ticks = sys_time.nb_sec_rem;
  gps_sirf.state.last_msg_time = sys_time.nb_sec;
  sirf_parse_msg();
  if (gps_sirf.msg_valid) {
    if (gps_sirf.state.fix == GPS_FIX_3D) {
      gps_sirf.state.last_3dfix_ticks = sys_time.nb_sec_rem;
      gps_sirf.state.last_3dfix_time = sys_time.nb_sec;
    }
    AbiSendMsgGPS(GPS_SIRF_ID, now_ts, &gps_sirf.state);
  }
  gps_sirf.msg_valid = false;
}

void sirf_parse_char(uint8_t c)
{
  switch (gps_sirf.read_state) {
    case UNINIT:
      if (c == 0xA0) {
        gps_sirf.msg_len = 0;
        gps_sirf.msg_buf[gps_sirf.msg_len] = c;
        gps_sirf.msg_len++;
        gps_sirf.read_state = GOT_A0;
      }
      break;
    case GOT_A0:
      if (c == 0xA2) {
        gps_sirf.msg_buf[gps_sirf.msg_len] = c;
        gps_sirf.msg_len++;
        gps_sirf.read_state = GOT_A2;
      } else {
        goto restart;
      }
      break;
    case GOT_A2:
      gps_sirf.msg_buf[gps_sirf.msg_len] = c;
      gps_sirf.msg_len++;
      if (c == 0xB0) {
        gps_sirf.read_state = GOT_B0;
      }
      break;
    case GOT_B0:
      if (c == 0xB3) {
        gps_sirf.msg_buf[gps_sirf.msg_len] = c;
        gps_sirf.msg_len++;
        gps_sirf.msg_available = true;
      } else {
        goto restart;
      }
      break;
    default:
      break;
  }
  return;

restart:
  gps_sirf.read_state = UNINIT;
}

int start_time = 0;
int ticks = 0;
int start_time2 = 0;
int ticks2 = 0;

void sirf_parse_41(void)
{
  struct sirf_msg_41 *p = (struct sirf_msg_41 *)&gps_sirf.msg_buf[4];

  gps_sirf.state.tow = Invert4Bytes(p->tow);
  gps_sirf.state.hmsl = Invert4Bytes(p->alt_msl) * 10;
  SetBit(gps_sirf.state.valid_fields, GPS_VALID_HMSL_BIT);
  gps_sirf.state.num_sv = p->num_sat;
  gps_sirf.state.nb_channels = p ->num_sat;

  /* read latitude, longitude and altitude from packet */
  gps_sirf.state.lla_pos.lat = Invert4Bytes(p->latitude);
  gps_sirf.state.lla_pos.lon = Invert4Bytes(p->longitude);
  gps_sirf.state.lla_pos.alt = Invert4Bytes(p->alt_ellipsoid) * 10;
  SetBit(gps_sirf.state.valid_fields, GPS_VALID_POS_LLA_BIT);

  gps_sirf.state.sacc = (Invert2Bytes(p->ehve) >> 16);
  gps_sirf.state.course = RadOfDeg(Invert2Bytes(p->cog)) * pow(10, 5);
  SetBit(gps_sirf.state.valid_fields, GPS_VALID_COURSE_BIT);
  gps_sirf.state.gspeed = RadOfDeg(Invert2Bytes(p->sog)) * pow(10, 5);
  gps_sirf.state.cacc = RadOfDeg(Invert2Bytes(p->heading_err)) * pow(10, 5);
  gps_sirf.state.pacc = Invert4Bytes(p->ehpe);
  gps_sirf.state.pdop = p->hdop * 20;

  if ((p->nav_type >> 8 & 0x7) >= 0x4) {
    gps_sirf.state.fix = GPS_FIX_3D;
  } else if ((p->nav_type >> 8 & 0x7) >= 0x1) {
    gps_sirf.state.fix = GPS_FIX_2D;
  } else {
    gps_sirf.state.fix = GPS_FIX_NONE;
  }

  gps_sirf.msg_valid = true;
}

void sirf_parse_2(void)
{
  struct sirf_msg_2 *p = (struct sirf_msg_2 *)&gps_sirf.msg_buf[4];

  gps_sirf.state.week = Invert2Bytes(p->week);

  gps_sirf.state.ecef_pos.x = Invert4Bytes(p->x_pos) * 100;
  gps_sirf.state.ecef_pos.y = Invert4Bytes(p->y_pos) * 100;
  gps_sirf.state.ecef_pos.z = Invert4Bytes(p->z_pos) * 100;
  SetBit(gps_sirf.state.valid_fields, GPS_VALID_POS_ECEF_BIT);

  gps_sirf.state.ecef_vel.x = (Invert2Bytes(p->vx) >> 16) * 100 / 8;
  gps_sirf.state.ecef_vel.y = (Invert2Bytes(p->vy) >> 16) * 100 / 8;
  gps_sirf.state.ecef_vel.z = (Invert2Bytes(p->vz) >> 16) * 100 / 8;
  SetBit(gps_sirf.state.valid_fields, GPS_VALID_VEL_ECEF_BIT);

  if (gps_sirf.state.fix == GPS_FIX_3D) {
    ticks++;
#if DEBUG_SIRF
    printf("GPS %i %i %i %i\n", ticks, (sys_time.nb_sec - start_time), ticks2, (sys_time.nb_sec - start_time2));
#endif
  } else if (sys_time.nb_sec - gps_sirf.state.last_3dfix_time > 10) {
    start_time = sys_time.nb_sec;
    ticks = 0;
  }

  gps_sirf.msg_valid = true;
}

void sirf_parse_msg(void)
{
  //Set msg_valid to false and check if it is a valid message
  gps_sirf.msg_valid = false;
  if (gps_sirf.msg_len < 8) {
    return;
  }

  if (start_time2 == 0) {
    start_time2 = sys_time.nb_sec;
  }
  ticks2++;

  //Check the message id and parse the message
  uint8_t message_id = gps_sirf.msg_buf[4];
  switch (message_id) {
    case 0x29:
      sirf_parse_41();
      break;
    case 0x02:
      sirf_parse_2();
      break;
  }

}

void gps_sirf_event(void)
{
  struct link_device *dev = &((SIRF_GPS_LINK).device);

  while (dev->char_available(dev->periph)) {
    sirf_parse_char(dev->get_byte(dev->periph));
    if (gps_sirf.msg_available) {
      gps_sirf_msg();
    }
  }
}
