/*
 *
 * Copyright (C) 2008-2011 The Paparazzi Team
 *               2014 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file gps_nmea.c
 * Parser for the NMEA protocol.
 *
 * TODO: THIS NMEA-PARSER IS NOT WELL TESTED AND INCOMPLETE!!!
 * Status:
 *  Parsing GGA and RMC is complete, GSA and other records are
 *  incomplete.
 */

#include "subsystems/gps.h"

#include "led.h"

#if GPS_USE_LATLONG
/* currently needed to get nav_utm_zone0 */
#include "subsystems/navigation/common_nav.h"
#endif
#include "math/pprz_geodetic_float.h"

#include <inttypes.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#ifndef NMEA_PRINT
#define NMEA_PRINT(...) {};
#endif

#if NMEA_PRINT == printf
#include <stdio.h>
#endif

struct GpsNmea gps_nmea;

static void nmea_parse_GSA(void);
static void nmea_parse_RMC(void);
static void nmea_parse_GGA(void);
static void nmea_parse_GSV(void);


void gps_impl_init(void)
{
  gps.nb_channels = GPS_NB_CHANNELS;
  gps_nmea.msg_available = FALSE;
  gps_nmea.pos_available = FALSE;
  gps_nmea.have_gsv = FALSE;
  gps_nmea.gps_nb_ovrn = 0;
  gps_nmea.msg_len = 0;
  nmea_parse_prop_init();
}

void gps_nmea_msg(void (* _cb)(void))
{
  gps.last_msg_ticks = sys_time.nb_sec_rem;
  gps.last_msg_time = sys_time.nb_sec;
  nmea_parse_msg();
  if (gps_nmea.pos_available) {
    if (gps.fix == GPS_FIX_3D) {
      gps.last_3dfix_ticks = sys_time.nb_sec_rem;
      gps.last_3dfix_time = sys_time.nb_sec;
    }
    _cb();
  }
  gps_nmea.msg_available = FALSE;
}

void WEAK nmea_parse_prop_init(void)
{
}

void WEAK nmea_parse_prop_msg(void)
{
}

/**
 * parse_nmea_char() has a complete line.
 * Find out what type of message it is and
 * hand it to the parser for that type.
 */
void nmea_parse_msg(void)
{

  if (gps_nmea.msg_len > 5 && !strncmp(&gps_nmea.msg_buf[2] , "RMC", 3)) {
    gps_nmea.msg_buf[gps_nmea.msg_len] = 0;
    NMEA_PRINT("RMC: \"%s\" \n\r", gps_nmea.msg_buf);
    nmea_parse_RMC();
  } else if (gps_nmea.msg_len > 5 && !strncmp(&gps_nmea.msg_buf[2] , "GGA", 3)) {
    gps_nmea.msg_buf[gps_nmea.msg_len] = 0;
    NMEA_PRINT("GGA: \"%s\" \n\r", gps_nmea.msg_buf);
    nmea_parse_GGA();
  } else if (gps_nmea.msg_len > 5 && !strncmp(&gps_nmea.msg_buf[2] , "GSA", 3)) {
    gps_nmea.msg_buf[gps_nmea.msg_len] = 0;
    NMEA_PRINT("GSA: \"%s\" \n\r", gps_nmea.msg_buf);
    nmea_parse_GSA();
  } else if (gps_nmea.msg_len > 5 && !strncmp(&gps_nmea.msg_buf[2] , "GSV", 3)) {
    gps_nmea.msg_buf[gps_nmea.msg_len] = 0;
    gps_nmea.have_gsv = TRUE;
    NMEA_PRINT("GSV: \"%s\" \n\r", gps_nmea.msg_buf);
    nmea_parse_GSV();
  } else {
    gps_nmea.msg_buf[gps_nmea.msg_len] = 0;
    NMEA_PRINT("Other/propriarty message: len=%i \n\r \"%s\" \n\r", gps_nmea.msg_len, gps_nmea.msg_buf);
    nmea_parse_prop_msg();
  }

  // reset message-buffer
  gps_nmea.msg_len = 0;
}


/**
 * This is the actual parser.
 * It reads one character at a time
 * setting gps_nmea.msg_available to TRUE
 * after a full line.
 */
void nmea_parse_char(uint8_t c)
{
  //reject empty lines
  if (gps_nmea.msg_len == 0) {
    if (c == '\r' || c == '\n' || c == '$') {
      return;
    }
  }

  // fill the buffer, unless it's full
  if (gps_nmea.msg_len < NMEA_MAXLEN - 1) {

    // messages end with a linefeed
    //AD: TRUNK:       if (c == '\r' || c == '\n')
    if (c == '\r' || c == '\n') {
      gps_nmea.msg_available = TRUE;
    } else {
      gps_nmea.msg_buf[gps_nmea.msg_len] = c;
      gps_nmea.msg_len ++;
    }
  }

  if (gps_nmea.msg_len >= NMEA_MAXLEN - 1) {
    gps_nmea.msg_available = TRUE;
  }
}

/**
 * Calculate control sum of binary buffer
 */
uint8_t nmea_calc_crc(const char *buff, int buff_sz)
{
  uint8_t chsum = 0,
          it;

  for (it = 0; it < buff_sz; ++it) {
    chsum ^= buff[it];
  }

  return chsum;
}

/**
 * parse GPGSA-nmea-messages stored in
 * nmea_msg_buf .
 */
static void nmea_parse_GSA(void)
{
  int i = 6;     // current position in the message, start after: GPGSA,

  // attempt to reject empty packets right away
  if (gps_nmea.msg_buf[i] == ',' && gps_nmea.msg_buf[i + 1] == ',') {
    NMEA_PRINT("p_GPGSA() - skipping empty message\n\r");
    return;
  }

  // get auto2D/3D
  // ignored
  nmea_read_until(&i);

  // get 2D/3D-fix
  // set gps_mode=3=3d, 2=2d, 1=no fix or 0
  gps.fix = atoi(&gps_nmea.msg_buf[i]);
  if (gps.fix == 1) {
    gps.fix = 0;
  }
  NMEA_PRINT("p_GPGSA() - gps.fix=%i (3=3D)\n\r", gps.fix);
  nmea_read_until(&i);

  // up to 12 PRNs of satellites used for fix
  int satcount = 0;
  int prn_cnt;
  for (prn_cnt = 0; prn_cnt < 12; prn_cnt++) {
    if (gps_nmea.msg_buf[i] != ',') {
      int prn = atoi(&gps_nmea.msg_buf[i]);
      NMEA_PRINT("p_GPGSA() - PRN %i=%i\n\r", satcount, prn);
      if (!gps_nmea.have_gsv) {
        gps.svinfos[prn_cnt].svid = prn;
      }
      satcount++;
    }
    else {
      if (!gps_nmea.have_gsv) {
        gps.svinfos[prn_cnt].svid = 0;
      }
    }
    nmea_read_until(&i);
  }

  // PDOP
  float pdop = strtof(&gps_nmea.msg_buf[i], NULL);
  gps.pdop = pdop * 100;
  NMEA_PRINT("p_GPGSA() - pdop=%f\n\r", pdop);
  nmea_read_until(&i);

  // HDOP
  float hdop __attribute__((unused)) = strtof(&gps_nmea.msg_buf[i], NULL);
  NMEA_PRINT("p_GPGSA() - hdop=%f\n\r", hdop);
  nmea_read_until(&i);

  // VDOP
  float vdop __attribute__((unused)) = strtof(&gps_nmea.msg_buf[i], NULL);
  NMEA_PRINT("p_GPGSA() - vdop=%f\n\r", vdop);
  nmea_read_until(&i);

}

/**
 * parse GPRMC-nmea-messages stored in
 * gps_nmea.msg_buf .
 */
static void nmea_parse_RMC(void)
{
  int i = 6;     // current position in the message, start after: GPRMC,

  // attempt to reject empty packets right away
  if (gps_nmea.msg_buf[i] == ',' && gps_nmea.msg_buf[i + 1] == ',') {
    NMEA_PRINT("p_GPRMC() - skipping empty message\n\r");
    return;
  }
  // First read time (ignored)

  // get warning
  nmea_read_until(&i);

  // get lat
  nmea_read_until(&i);

  // get North/South
  nmea_read_until(&i);

  // get lon
  nmea_read_until(&i);

  // get eath/west
  nmea_read_until(&i);

  // get speed
  nmea_read_until(&i);
  double speed = strtod(&gps_nmea.msg_buf[i], NULL);
  gps.gspeed = speed * 1.852 * 100 / (60 * 60);
  NMEA_PRINT("p_GPRMC() - ground-speed=%f knot = %d cm/s\n\r", (speed * 1000), (gps.gspeed * 1000));

  // get course
  nmea_read_until(&i);
  double course = strtod(&gps_nmea.msg_buf[i], NULL);
  gps.course = RadOfDeg(course) * 1e7;
  NMEA_PRINT("COURSE: %d \n\r", gps.course);
}


/**
 * parse GPGGA-nmea-messages stored in
 * gps_nmea.msg_buf .
 */
static void nmea_parse_GGA(void)
{
  int i = 6;     // current position in the message, start after: GPGGA,
  double degrees, minutesfrac;
  struct LlaCoor_f lla_f;

  // attempt to reject empty packets right away
  if (gps_nmea.msg_buf[i] == ',' && gps_nmea.msg_buf[i + 1] == ',') {
    NMEA_PRINT("p_GPGGA() - skipping empty message\n\r");
    return;
  }

  // get UTC time [hhmmss.sss]
  // ignored GpsInfo.PosLLA.TimeOfFix.f = strtod(&packet[i], NULL);
  // FIXME: parse UTC time correctly
  double time = strtod(&gps_nmea.msg_buf[i], NULL);
  gps.tow = (uint32_t)((time + 1) * 1000);

  // get latitude [ddmm.mmmmm]
  nmea_read_until(&i);
  double lat = strtod(&gps_nmea.msg_buf[i], NULL);
  // convert to pure degrees [dd.dddd] format
  minutesfrac = modf(lat / 100, &degrees);
  lat = degrees + (minutesfrac * 100) / 60;

  // get latitute N/S
  nmea_read_until(&i);
  if (gps_nmea.msg_buf[i] == 'S') {
    lat = -lat;
  }

  // convert to radians
  lla_f.lat = RadOfDeg(lat);
  gps.lla_pos.lat = lat * 1e7; // convert to fixed-point
  NMEA_PRINT("p_GPGGA() - lat=%f gps_lat=%f\n\r", (lat * 1000), lla_f.lat);


  // get longitude [ddmm.mmmmm]
  nmea_read_until(&i);
  double lon = strtod(&gps_nmea.msg_buf[i], NULL);
  // convert to pure degrees [dd.dddd] format
  minutesfrac = modf(lon / 100, &degrees);
  lon = degrees + (minutesfrac * 100) / 60;

  // get longitude E/W
  nmea_read_until(&i);
  if (gps_nmea.msg_buf[i] == 'W') {
    lon = -lon;
  }

  // convert to radians
  lla_f.lon = RadOfDeg(lon);
  gps.lla_pos.lon = lon * 1e7; // convert to fixed-point
  NMEA_PRINT("p_GPGGA() - lon=%f gps_lon=%f time=%u\n\r", (lon * 1000), lla_f.lon, gps.tow);

  // get position fix status
  nmea_read_until(&i);
  // 0 = Invalid, 1 = Valid SPS, 2 = Valid DGPS, 3 = Valid PPS
  // check for good position fix
  if ((gps_nmea.msg_buf[i] != '0') && (gps_nmea.msg_buf[i] != ','))  {
    gps_nmea.pos_available = TRUE;
    NMEA_PRINT("p_GPGGA() - POS_AVAILABLE == TRUE\n\r");
  } else {
    gps_nmea.pos_available = FALSE;
    NMEA_PRINT("p_GPGGA() - gps_pos_available == false\n\r");
  }

  // get number of satellites used in GPS solution
  nmea_read_until(&i);
  gps.num_sv = atoi(&gps_nmea.msg_buf[i]);
  NMEA_PRINT("p_GPGGA() - gps_numSatlitesUsed=%i\n\r", gps.num_sv);

  // get HDOP, but we use PDOP from GSA message
  nmea_read_until(&i);
  //float hdop = strtof(&gps_nmea.msg_buf[i], NULL);
  //gps.pdop = hdop * 100;

  // get altitude (in meters) above geoid (MSL)
  nmea_read_until(&i);
  float hmsl = strtof(&gps_nmea.msg_buf[i], NULL);
  gps.hmsl = hmsl * 1000;
  NMEA_PRINT("p_GPGGA() - gps.hmsl=%i\n\r", gps.hmsl);

  // get altitude units (always M)
  nmea_read_until(&i);

  // get geoid seperation
  nmea_read_until(&i);
  float geoid = strtof(&gps_nmea.msg_buf[i], NULL);
  NMEA_PRINT("p_GPGGA() - geoid alt=%f\n\r", geoid);
  // height above ellipsoid
  lla_f.alt = hmsl + geoid;
  gps.lla_pos.alt = lla_f.alt * 1000;
  NMEA_PRINT("p_GPGGA() - gps.alt=%i\n\r", gps.lla_pos.alt);

  // get seperations units
  nmea_read_until(&i);
  // get DGPS age
  nmea_read_until(&i);
  // get DGPS station ID

#if GPS_USE_LATLONG
  /* convert to utm */
  struct UtmCoor_f utm_f;
  utm_f.zone = nav_utm_zone0;
  utm_of_lla_f(&utm_f, &lla_f);

  /* copy results of utm conversion */
  gps.utm_pos.east = utm_f.east * 100;
  gps.utm_pos.north = utm_f.north * 100;
  gps.utm_pos.alt = gps.lla_pos.alt;
  gps.utm_pos.zone = nav_utm_zone0;
#endif

  /* convert to ECEF */
  struct EcefCoor_f ecef_f;
  ecef_of_lla_f(&ecef_f, &lla_f);
  gps.ecef_pos.x = ecef_f.x * 100;
  gps.ecef_pos.y = ecef_f.y * 100;
  gps.ecef_pos.z = ecef_f.z * 100;
}

/**
 * Parse GSV-nmea-messages.
 * Msg stored in gps_nmea.msg_buf.
 */
static void nmea_parse_GSV(void)
{
  int i = 6;     // current position in the message, start after: GxGSA,

  // attempt to reject empty packets right away
  if (gps_nmea.msg_buf[i] == ',' && gps_nmea.msg_buf[i + 1] == ',') {
    NMEA_PRINT("p_GSV() - skipping empty message\n\r");
    return;
  }

  // check what satellites this messages contains
  // GPGSV -> GPS
  // GLGSV -> GLONASS
  bool_t is_glonass = FALSE;
  if (!strncmp(&gps_nmea.msg_buf[0] , "GL", 2)) {
    is_glonass = TRUE;
  }

  // total sentences
  int nb_sen __attribute__((unused)) = atoi(&gps_nmea.msg_buf[i]);
  NMEA_PRINT("p_GSV() - %i sentences\n\r", nb_sen);
  nmea_read_until(&i);

  // current sentence
  int cur_sen = atoi(&gps_nmea.msg_buf[i]);
  NMEA_PRINT("p_GSV() - sentence=%i\n\r", cur_sen);
  nmea_read_until(&i);

  // num satellites in view
  int num_sat __attribute__((unused)) = atoi(&gps_nmea.msg_buf[i]);
  NMEA_PRINT("p_GSV() - num_sat=%i\n\r", num_sat);
  nmea_read_until(&i);

  // up to 4 sats per sentence
  int sat_cnt;
  for (sat_cnt = 0; sat_cnt < 4; sat_cnt++) {
    if (gps_nmea.msg_buf[i] == ',') break;
    // 4 fields per sat: PRN, elevation (deg), azimuth (deg), SNR
    int prn = atoi(&gps_nmea.msg_buf[i]);
    nmea_read_until(&i);
    int elev = atoi(&gps_nmea.msg_buf[i]);
    nmea_read_until(&i);
    int azim = atoi(&gps_nmea.msg_buf[i]);
    nmea_read_until(&i);
    int snr = atoi(&gps_nmea.msg_buf[i]);
    nmea_read_until(&i);

    int ch_idx = (cur_sen - 1) * 4 + sat_cnt;
    // don't populate svinfos with GLONASS sats for now
    if (!is_glonass && ch_idx > 0 && ch_idx < 12) {
      gps.svinfos[ch_idx].svid = prn;
      gps.svinfos[ch_idx].cno = snr;
      gps.svinfos[ch_idx].elev = elev;
      gps.svinfos[ch_idx].azim = azim;
    }
    if (is_glonass) {
      NMEA_PRINT("p_GSV() - GLONASS %i PRN=%i elev=%i azim=%i snr=%i\n\r", ch_idx, prn, elev, azim, snr);
    }
    else {
      NMEA_PRINT("p_GSV() - GPS %i PRN=%i elev=%i azim=%i snr=%i\n\r", ch_idx, prn, elev, azim, snr);
    }
  }
}
