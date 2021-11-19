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
 * Basic parser for the NMEA protocol.
 *
 * Status:
 *  Parsing GGA, RMC, GSA and GSV.
 */

#include "gps_nmea.h"
#include "modules/gps/gps.h"
#include "modules/core/abi.h"
#include "led.h"

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

/* line parser status */
#define WAIT          0
#define GOT_START     1
#define GOT_CHECKSUM  2
#define GOT_END       3

struct GpsNmea gps_nmea;

static bool nmea_parse_GSA(void);
static bool nmea_parse_RMC(void);
static bool nmea_parse_GGA(void);
static bool nmea_parse_GSV(void);

void gps_nmea_init(void)
{
  gps_nmea.state.nb_channels = GPS_NMEA_NB_CHANNELS;
  gps_nmea.is_configured = false;
  gps_nmea.msg_available = false;
  gps_nmea.have_gsv = false;
  gps_nmea.gps_nb_ovrn = 0;
  gps_nmea.msg_len = 0;
  nmea_parse_prop_init();
  nmea_configure();
}

void gps_nmea_event(void)
{
  struct link_device *dev = &((NMEA_GPS_LINK).device);

  if (!gps_nmea.is_configured) {
    nmea_configure();
    return;
  }
  while (dev->char_available(dev->periph)) {
    nmea_parse_char(dev->get_byte(dev->periph));
    if (gps_nmea.msg_available) {
      nmea_gps_msg();
      gps_nmea.msg_available = false;
    }
  }
}

void nmea_gps_msg(void)
{
  // current timestamp
  uint32_t now_ts = get_sys_time_usec();

  gps_nmea.state.last_msg_ticks = sys_time.nb_sec_rem;
  gps_nmea.state.last_msg_time = sys_time.nb_sec;

  /* if a message was a valid/supported sentence, send update */
  if (nmea_parse_msg()) {
    if (gps_nmea.state.fix == GPS_FIX_3D) {
      gps_nmea.state.last_3dfix_ticks = sys_time.nb_sec_rem;
      gps_nmea.state.last_3dfix_time = sys_time.nb_sec;
    }
    AbiSendMsgGPS(GPS_NMEA_ID, now_ts, &gps_nmea.state);
  }
}

void WEAK nmea_configure(void)
{
  gps_nmea.is_configured = true;
}

void WEAK nmea_parse_prop_init(void)
{
}

bool WEAK nmea_parse_prop_msg(void)
{
  return false;
}

/**
 * nmea_parse_char() has a complete line.
 * Find out what type of message it is and
 * hand it to the parser for that type.
 * @return true if msg was valid and gps_nmea.state updated
 */
bool nmea_parse_msg(void)
{
  bool msg_valid = false;

  if (gps_nmea.msg_len > 5 && !strncmp(&gps_nmea.msg_buf[2] , "RMC", 3)) {
    gps_nmea.msg_buf[gps_nmea.msg_len] = 0;
    NMEA_PRINT("RMC: \"%s\" \n\r", gps_nmea.msg_buf);
    msg_valid = nmea_parse_RMC();
  } else if (gps_nmea.msg_len > 5 && !strncmp(&gps_nmea.msg_buf[2] , "GGA", 3)) {
    gps_nmea.msg_buf[gps_nmea.msg_len] = 0;
    NMEA_PRINT("GGA: \"%s\" \n\r", gps_nmea.msg_buf);
    msg_valid = nmea_parse_GGA();
  } else if (gps_nmea.msg_len > 5 && !strncmp(&gps_nmea.msg_buf[2] , "GSA", 3)) {
    gps_nmea.msg_buf[gps_nmea.msg_len] = 0;
    NMEA_PRINT("GSA: \"%s\" \n\r", gps_nmea.msg_buf);
    msg_valid = nmea_parse_GSA();
  } else if (gps_nmea.msg_len > 5 && !strncmp(&gps_nmea.msg_buf[2] , "GSV", 3)) {
    gps_nmea.msg_buf[gps_nmea.msg_len] = 0;
    gps_nmea.have_gsv = true;
    NMEA_PRINT("GSV: \"%s\" \n\r", gps_nmea.msg_buf);
    msg_valid = nmea_parse_GSV();
  } else {
    gps_nmea.msg_buf[gps_nmea.msg_len] = 0;
    NMEA_PRINT("Other/propriarty message: len=%i \n\r \"%s\" \n\r", gps_nmea.msg_len, gps_nmea.msg_buf);
    msg_valid = nmea_parse_prop_msg();
  }

  // reset line parser
  gps_nmea.status = WAIT;

  /* indicate if msg was valid/supported and gps_nmea.state updated */
  return msg_valid;
}


/**
 * This is the actual parser.
 * It reads one character at a time
 * setting gps_nmea.msg_available to TRUE
 * after a full line.
 */
void nmea_parse_char(uint8_t c)
{

  switch (gps_nmea.status) {
    case WAIT:
      gps_nmea.msg_len = 0;
      /* valid message needs to start with dollar sign */
      if (c == '$') {
        gps_nmea.status = GOT_START;
      }
      break;

    case GOT_START:
      switch (c) {
        case '\r':
        case '\n':
          if (gps_nmea.msg_len == 0) {
            //reject empty lines
            gps_nmea.status = WAIT;
            break;
          }
          else {
            // TODO: check for CRC before setting msg as available
            gps_nmea.status = GOT_END;
            gps_nmea.msg_available = true;
          }
          break;

        case '$':
          // got another dollar sign, msg incomplete: reset
          gps_nmea.msg_buf[gps_nmea.msg_len] = 0;
          NMEA_PRINT("nmea_parse_char: skipping incomplete msg: len=%i, \"%s\"\n\r",
                     gps_nmea.msg_len, gps_nmea.msg_buf);
          gps_nmea.status = WAIT;
          break;

        default:
           // fill the buffer, unless it's full
          if (gps_nmea.msg_len < NMEA_MAXLEN - 1) {
            gps_nmea.msg_buf[gps_nmea.msg_len] = c;
            gps_nmea.msg_len++;
          }
          else {
            gps_nmea.msg_buf[gps_nmea.msg_len] = 0;
            NMEA_PRINT("nmea_parse_char: msg too long, len=%i, \"%s\"\n\r",
                       gps_nmea.msg_len, gps_nmea.msg_buf);
            gps_nmea.status = WAIT;
          }
          break;
      }
      break;

    case GOT_CHECKSUM:
      // TODO
      break;

    case GOT_END:
      // shouldn't really happen, msg should be parsed and state reset before the next char
      NMEA_PRINT("nmea_parse_char: this should not happen!");
      break;

    default:
      break;
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
 * Parse GSA NMEA messages.
 * GPS DOP and active satellites.
 * Msg stored in gps_nmea.msg_buf.
 * @return true if msg was valid and gps_nmea.state updated
 */
static bool nmea_parse_GSA(void)
{
  int i = 6;     // current position in the message, start after: GPGSA,

  // attempt to reject empty packets right away
  if (gps_nmea.msg_buf[i] == ',' && gps_nmea.msg_buf[i + 1] == ',') {
    NMEA_PRINT("p_GSA() - skipping empty message\n\r");
    return false;
  }

  // get auto2D/3D
  // ignored
  nmea_read_until(&i);

  // get 2D/3D-fix
  // set gps_mode=3=3d, 2=2d, 1=no fix or 0
  gps_nmea.state.fix = atoi(&gps_nmea.msg_buf[i]);
  if (gps_nmea.state.fix == 1) {
    gps_nmea.state.fix = 0;
  }
  NMEA_PRINT("p_GSA() - gps_nmea.state.fix=%i (3=3D)\n\r", gps_nmea.state.fix);
  nmea_read_until(&i);

  // up to 12 PRNs of satellites used for fix
  int satcount = 0;
  int prn_cnt;
  for (prn_cnt = 0; prn_cnt < 12; prn_cnt++) {
    if (gps_nmea.msg_buf[i] != ',') {
      int prn = atoi(&gps_nmea.msg_buf[i]);
      NMEA_PRINT("p_GSA() - PRN %i=%i\n\r", satcount, prn);
      if (!gps_nmea.have_gsv) {
        gps_nmea.state.svinfos[prn_cnt].svid = prn;
      }
      satcount++;
    }
    else {
      if (!gps_nmea.have_gsv) {
        gps_nmea.state.svinfos[prn_cnt].svid = 0;
      }
    }
    nmea_read_until(&i);
  }

  // PDOP
  float pdop = strtof(&gps_nmea.msg_buf[i], NULL);
  gps_nmea.state.pdop = pdop * 100;
  NMEA_PRINT("p_GSA() - pdop=%f\n\r", pdop);
  nmea_read_until(&i);

  // HDOP
  float hdop __attribute__((unused)) = strtof(&gps_nmea.msg_buf[i], NULL);
  NMEA_PRINT("p_GSA() - hdop=%f\n\r", hdop);
  nmea_read_until(&i);

  // VDOP
  float vdop __attribute__((unused)) = strtof(&gps_nmea.msg_buf[i], NULL);
  NMEA_PRINT("p_GSA() - vdop=%f\n\r", vdop);
  nmea_read_until(&i);

  /* indicate that msg was valid and gps_nmea.state updated */
  return true;
}

/**
 * Parse RMC NMEA messages.
 * Recommended minimum GPS sentence.
 * Msg stored in gps_nmea.msg_buf.
 * @return true if msg was valid and gps_nmea.state updated
 */
static bool nmea_parse_RMC(void)
{
  int i = 6;     // current position in the message, start after: GPRMC,

  // attempt to reject empty packets right away
  if (gps_nmea.msg_buf[i] == ',' && gps_nmea.msg_buf[i + 1] == ',') {
    NMEA_PRINT("p_RMC() - skipping empty message\n\r");
    return false;
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
  gps_nmea.state.gspeed = speed * 1.852 * 100 / (60 * 60);
  NMEA_PRINT("p_RMC() - ground-speed=%f knot = %d cm/s\n\r", speed, (gps_nmea.state.gspeed * 1000));

  // get course
  nmea_read_until(&i);
  double course = strtod(&gps_nmea.msg_buf[i], NULL);
  gps_nmea.state.course = RadOfDeg(course) * 1e7;
  NMEA_PRINT("p_RMC() - course: %f deg\n\r", course);
  SetBit(gps_nmea.state.valid_fields, GPS_VALID_COURSE_BIT);

  /* indicate that msg was valid and gps_nmea.state updated */
  return true;
}


/**
 * Parse GGA NMEA messages.
 * GGA has essential fix data providing 3D location and HDOP.
 * Msg stored in gps_nmea.msg_buf.
 * @return true if msg was valid and gps_nmea.state updated
 */
static bool nmea_parse_GGA(void)
{
  int i = 6;     // current position in the message, start after: GPGGA,
  double degrees, minutesfrac;
  struct LlaCoor_f lla_f;

  // attempt to reject empty packets right away
  if (gps_nmea.msg_buf[i] == ',' && gps_nmea.msg_buf[i + 1] == ',') {
    NMEA_PRINT("p_GGA() - skipping empty message\n\r");
    return false;
  }

  // get UTC time [hhmmss.sss]
  // ignored GpsInfo.PosLLA.TimeOfFix.f = strtod(&packet[i], NULL);
  // FIXME: parse UTC time correctly
  double utc_time = strtod(&gps_nmea.msg_buf[i], NULL);
  gps_nmea.state.tow = (uint32_t)((utc_time + 1) * 1000);

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
  gps_nmea.state.lla_pos.lat = lat * 1e7; // convert to fixed-point
  NMEA_PRINT("p_GGA() - lat=%f gps_lat=%f\n\r", (lat * 1000), lla_f.lat);


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
  gps_nmea.state.lla_pos.lon = lon * 1e7; // convert to fixed-point
  NMEA_PRINT("p_GGA() - lon=%f gps_lon=%f time=%u\n\r", (lon * 1000), lla_f.lon, gps_nmea.state.tow);
  SetBit(gps_nmea.state.valid_fields, GPS_VALID_POS_LLA_BIT);

  // get position fix status
  nmea_read_until(&i);
  // 0 = Invalid, 1 = Valid SPS, 2 = Valid DGPS, 3 = Valid PPS
  // check for good position fix
  if ((gps_nmea.msg_buf[i] != '0') && (gps_nmea.msg_buf[i] != ','))  {
    NMEA_PRINT("p_GGA() - POS_AVAILABLE == TRUE\n\r");
  } else {
    NMEA_PRINT("p_GGA() - gps_pos_available == false\n\r");
  }

  // get number of satellites used in GPS solution
  nmea_read_until(&i);
  gps_nmea.state.num_sv = atoi(&gps_nmea.msg_buf[i]);
  NMEA_PRINT("p_GGA() - gps_numSatlitesUsed=%i\n\r", gps_nmea.state.num_sv);

  // get HDOP, but we use PDOP from GSA message
  nmea_read_until(&i);
  //float hdop = strtof(&gps_nmea.msg_buf[i], NULL);
  //gps_nmea.state.pdop = hdop * 100;

  // get altitude (in meters) above geoid (MSL)
  nmea_read_until(&i);
  float hmsl = strtof(&gps_nmea.msg_buf[i], NULL);
  gps_nmea.state.hmsl = hmsl * 1000;
  NMEA_PRINT("p_GGA() - gps_nmea.state.hmsl=%i\n\r", gps_nmea.state.hmsl);
  SetBit(gps_nmea.state.valid_fields, GPS_VALID_HMSL_BIT);

  // get altitude units (always M)
  nmea_read_until(&i);

  // get geoid seperation
  nmea_read_until(&i);
  float geoid = strtof(&gps_nmea.msg_buf[i], NULL);
  NMEA_PRINT("p_GGA() - geoid alt=%f\n\r", geoid);
  // height above ellipsoid
  lla_f.alt = hmsl + geoid;
  gps_nmea.state.lla_pos.alt = lla_f.alt * 1000;
  NMEA_PRINT("p_GGA() - gps_nmea.state.alt=%i\n\r", gps_nmea.state.lla_pos.alt);

  // get seperations units
  nmea_read_until(&i);
  // get DGPS age
  nmea_read_until(&i);
  // get DGPS station ID

  /* convert to ECEF */
  struct EcefCoor_f ecef_f;
  ecef_of_lla_f(&ecef_f, &lla_f);
  gps_nmea.state.ecef_pos.x = ecef_f.x * 100;
  gps_nmea.state.ecef_pos.y = ecef_f.y * 100;
  gps_nmea.state.ecef_pos.z = ecef_f.z * 100;
  SetBit(gps_nmea.state.valid_fields, GPS_VALID_POS_ECEF_BIT);

  /* indicate that msg was valid and gps_nmea.state updated */
  return true;
}

/**
 * Parse GSV-nmea-messages.
 * Msg stored in gps_nmea.msg_buf.
 * @return true if msg was valid and gps_nmea.state updated
 */
static bool nmea_parse_GSV(void)
{
  int i = 6;     // current position in the message, start after: GxGSA,

  // attempt to reject empty packets right away
  if (gps_nmea.msg_buf[i] == ',' && gps_nmea.msg_buf[i + 1] == ',') {
    NMEA_PRINT("p_GSV() - skipping empty message\n\r");
    return false;
  }

  // check what satellites this messages contains
  // GPGSV -> GPS
  // GLGSV -> GLONASS
  bool is_glonass = false;
  if (!strncmp(&gps_nmea.msg_buf[0] , "GL", 2)) {
    is_glonass = true;
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
      gps_nmea.state.svinfos[ch_idx].svid = prn;
      gps_nmea.state.svinfos[ch_idx].cno = snr;
      gps_nmea.state.svinfos[ch_idx].elev = elev;
      gps_nmea.state.svinfos[ch_idx].azim = azim;
    }
    if (is_glonass) {
      NMEA_PRINT("p_GSV() - GLONASS %i PRN=%i elev=%i azim=%i snr=%i\n\r", ch_idx, prn, elev, azim, snr);
    }
    else {
      NMEA_PRINT("p_GSV() - GPS %i PRN=%i elev=%i azim=%i snr=%i\n\r", ch_idx, prn, elev, azim, snr);
    }
  }

  /* indicate that msg was valid and gps_nmea.state updated */
  return true;
}
