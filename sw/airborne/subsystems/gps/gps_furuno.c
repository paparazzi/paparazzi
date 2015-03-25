/*
 *
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file subsystems/gps/gps_furuno.c
 * GPS furuno based NMEA parser
 */

#include "gps_nmea.h"
#include "subsystems/gps.h"
#include <stdio.h>
#include <string.h>

#define GPS_FURUNO_SETTINGS_NB    10
static const char *gps_furuno_settings[GPS_FURUNO_SETTINGS_NB] = {
  "PERDAPI,FIXPERSEC,5",        // Receive position every 5 Hz
  "PERDAPI,CROUT,ALLOFF",       // Disable all propriarty output
  "PERDCFG,NMEAOUT,GGA,1",      // Enable GGA every fix
  "PERDCFG,NMEAOUT,RMC,1",      // Enable RMC every fix
  "PERDCFG,NMEAOUT,GSA,1",      // Enable GSA every fix
  "PERDCFG,NMEAOUT,GNS,0",      // Disable GNS
  "PERDCFG,NMEAOUT,ZDA,0",      // Disable ZDA
  "PERDCFG,NMEAOUT,GSV,0",      // Disable GSV
  "PERDCFG,NMEAOUT,GST,0",      // Disable GST
  "PERDAPI,CROUT,V"             // Enable raw velocity
};

static void nmea_parse_perdcrv(void);

#define GpsLinkDevice (&(GPS_LINK).device)

void nmea_parse_prop_init(void)
{
  static uint8_t i = 0;
  uint8_t j, len, crc;
  char buf[128];

  // Return when doen
  if (i == GPS_FURUNO_SETTINGS_NB) {
    return;
  }

  for (; i < GPS_FURUNO_SETTINGS_NB; i++) {
    len = strlen(gps_furuno_settings[i]);
    crc = nmea_calc_crc(gps_furuno_settings[i], len);
    sprintf(buf, "$%s*%02X\r\n", gps_furuno_settings[i], crc);

    // Check if there is enough space to send the config msg
    if (GpsLinkDevice->check_free_space(GpsLinkDevice->periph, len + 6)) {
      for (j = 0; j < len + 6; j++) {
        GpsLinkDevice->put_byte(GpsLinkDevice->periph, buf[j]);
      }
    } else {
      break;
    }
  }
}

void nmea_parse_prop_msg(void)
{
  if (gps_nmea.msg_len > 5 && !strncmp(gps_nmea.msg_buf , "PERDCRV", 7)) {
    nmea_parse_perdcrv();
  }
}

void nmea_parse_perdcrv(void)
{
  int i = 8;

  // Ignore reserved
  nmea_read_until(&i);

  // Ignore reserved
  nmea_read_until(&i);

  //EAST VEL
  double east_vel = strtod(&gps_nmea.msg_buf[i], NULL);
  gps.ned_vel.y = east_vel * 100; // in cm/s

  // Ignore reserved
  nmea_read_until(&i);

  // NORTH VEL
  double north_vel = strtod(&gps_nmea.msg_buf[i], NULL);
  gps.ned_vel.x = north_vel * 100; // in cm/s

  //Convert velocity to ecef
  struct LtpDef_i ltp;
  ltp_def_from_ecef_i(&ltp, &gps.ecef_pos);
  ecef_of_ned_vect_i(&gps.ecef_vel, &ltp, &gps.ned_vel);
}
