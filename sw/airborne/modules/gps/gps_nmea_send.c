/*
 * Copyright (C) 2025 Jean-Baptiste FORESTIER <jean-baptiste.forestier@enac.fr>
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/gps/gps_nmea_send.c"
 * @author Jean-Baptiste FORESTIER
 * @brief module used to send GPS data over a Tawaki UART for extern instrument using NMEA protocol
 * Exemple of use : MAPIR camera stores GPS data in metadata on each frame
 */



#include "modules/gps/gps_nmea_send.h"
#include "mcu_periph/uart.h"
#include "generated/airframe.h"
#include "state.h" // State interface for rotation compensation
#include "modules/gps/gps.h"

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <inttypes.h>

// ** Declaration ** //
struct Gps_Nmea_Send gps_nmea_send;

static void recover_gps_data(void);
static void build_nmea_sentence(void);
static uint8_t nmea_checksum(const char *sentence, int length);
static void nmea_convert_deg_to_DDMM(double deg, char *buf, int is_lat);
static void gps_nmea_get_system_date_str(char *buf_date, size_t buf_date_size, char *buf_time, size_t buf_time_size);
static void nmea_send(const char *payload, int payload_length);


/**
 * Initialization function
 */
void gps_nmea_send_init(void)
{
  gps_nmea_send.error_init = false;

  gps_nmea_send.msg.lat = 0;
  gps_nmea_send.msg.lon = 0;
  gps_nmea_send.msg.num_sv = 0;
  gps_nmea_send.msg.pdop = 0;
  gps_nmea_send.msg.hmsl = 0;
  gps_nmea_send.msg.vground = 0;
  gps_nmea_send.msg.course = 0;
}

/**
 * Periodic function to send data
 */
void gps_nmea_send_periodic(void)
{
  recover_gps_data();
  build_nmea_sentence();
}


/**
 * Function to recover data available onboard from GPS
 */
void recover_gps_data(void)
{
#if GPS_NMEA_SEND_USE_STATE_DATA
  struct LlaCoor_f *lla = stateGetPositionLla_f();
  gps_nmea_send.msg.lat = (double)gps->lat;
  gps_nmea_send.msg.lon = (double)gps->lon;
  gps_nmea_send.msg.hmsl = stateGetHmslOrigin_f() + stateGetPositionEnu_f()->z;
  gps_nmea_send.msg.vground = stateGetHorizontalSpeedNorm_f();
  gps_nmea_send.msg.course = NormCourseRad(stateGetHorizontalSpeedDir_f());
#else // Default, use unfiltered GPS data
  gps_nmea_send.msg.lat = (double)gps.lla_pos.lat / 1e7;
  gps_nmea_send.msg.lon = (double)gps.lla_pos.lon / 1e7;
  gps_nmea_send.msg.hmsl = (float)gps.hmsl / 1000.f; //in meters
  gps_nmea_send.msg.vground = (float)gps.gspeed / 100.f; // in meters
  gps_nmea_send.msg.course = (float)DegOfRad(gps.course / 1e7);
#endif
  gps_nmea_send.msg.num_sv = gps.num_sv;
  gps_nmea_send.msg.pdop = gps.pdop;
}

/**
 * Function to build sentence for NMEA Protocol
 */
void build_nmea_sentence(void)
{
  // Latitude and Longitude
  char lat_buf[16], lon_buf[16], gga[120], rmc[120], date_sys[6], time_sys[6];
  int len_gga = 0;
  int len_rmc = 0;
  uint8_t cs;
  nmea_convert_deg_to_DDMM(gps_nmea_send.msg.lat, lat_buf, 1);
  nmea_convert_deg_to_DDMM(gps_nmea_send.msg.lon, lon_buf, 0);
  char lat_hemi = (gps_nmea_send.msg.lat >= 0) ? 'N' : 'S';
  char lon_hemi = (gps_nmea_send.msg.lon >= 0) ? 'E' : 'W';

  for (int i = 0; i < 120; i++) { gga[i] = '\0'; rmc[i] = '\0'; }

  if (gps.fix >= GPS_FIX_3D) {
    gps_nmea_get_system_date_str(date_sys, sizeof(date_sys) + 1, time_sys, sizeof(time_sys) + 1);

    // -------------------------------
    //  GPGGA Frame
    // -------------------------------
    sprintf(gga,
            "$GPGGA,%s,%s,%c,%s,%c,1,%02u,%04u,%09.3f,M,0,M,,",
            time_sys, lat_buf, lat_hemi, lon_buf, lon_hemi,
            gps_nmea_send.msg.num_sv, gps_nmea_send.msg.pdop,
            gps_nmea_send.msg.hmsl);
    len_gga = 7;   // $GPGGA, = 7
    len_gga += 7;  // time_nmea, = 7
    len_gga += 10; // lat_buf = 9 + comma
    len_gga += 2;  // lat_hemi = N,
    len_gga += 11; // lon_buf = 10 + comma
    len_gga += 2;  // lon_hemi = W,
    len_gga += 2;  // GPS Quality indicator GPS fix = 1,
    len_gga += 3;  // number of sats = 08,
    len_gga += 5;  // position dilution of precision scaled by 100 = 1020,
    len_gga += 10; // height above mean sea level (MSL) in m = 02945.127,
    len_gga += 2;  // height above mean sea level unit = M,
    len_gga += 2;  // Ellipsoid to geoid distance set 0, don't where to find it = 0,
    len_gga += 2;  // Distance above unit = M,
    len_gga += 1;  // Empty = ,

    cs = nmea_checksum(gga, len_gga);
    sprintf(gga + len_gga, "*%02X\r\n", cs);
    len_gga += 5;  // * + Checksum + \r\n = *4F\r\n

    nmea_send(gga, len_gga);

    // -------------------------------
    //  GPRMC Frame
    // -------------------------------
    sprintf(rmc,
            "$GPRMC,%s,A,%s,%c,%s,%c,%05.1f,%05.1f,%s,000.0,W",
            time_sys, lat_buf, lat_hemi, lon_buf, lon_hemi,
            gps_nmea_send.msg.vground, gps_nmea_send.msg.course, date_sys);
    len_rmc = 7;   // $GPRMC, = 7
    len_rmc += 7;  // time_nmea, = 7
    len_rmc += 2;  // Status A=active or V=void = A,
    len_rmc += 10; // lat_buf = 9 + comma
    len_rmc += 2;  // lat_hemi = N,
    len_rmc += 11; // lon_buf = 10 + comma
    len_rmc += 2;  // lon_hemi = W,
    len_rmc += 6;  // vground = 050.4,
    len_rmc += 6;  // track angle course = 050.4,
    len_rmc += 7;  // Date = 230394,
    len_rmc += 7;  // Magnetic variation, in degrees = 003.1,W,

    cs = nmea_checksum(rmc, len_rmc);
    sprintf(rmc + len_rmc, "*%02X\r\n", cs);
    len_rmc += 5;  // * + Checksum + \r\n = *4F\r\n

    nmea_send(rmc, len_rmc);

  } else {
    //If no fix, empty GGA
    sprintf(gga, "$GPGGA,,,,,,0,00,99.99,,,,,,*68\r\n");
    nmea_send(gga, 33);

    //If no fix, empty RMC
    sprintf(rmc, "$GPRMC,,V,,,,,,,,,,*53\r\n");
    nmea_send(rmc, 24);
  }
}

/**
 * Function to calculate checksum
 */
uint8_t nmea_checksum(const char *sentence, int length)
{
  uint8_t cs = 0;
  for (int i = 0; i < length; i++) {
    cs ^= (uint8_t)sentence[i];
  }
  return cs;
}

/**
 * Function to convert lat long in DDLL.MMMM Format
 * Lat and long don't have the same length
 */
void nmea_convert_deg_to_DDMM(double deg, char *buf, int is_lat)
{
  double abs_deg = fabs(deg);
  int d = (int)abs_deg;
  double minutes = (abs_deg - d) * 60.0;

  if (is_lat) {
    // 2 integers for degrees 0 to 90 without sign => %02d | 2 integers for minutes, 1 for point, 4 for minutes decimals
    sprintf(buf, "%02d%07.4f", d, minutes);
  } else {
    // 3 integers for degrees 0 to 180 without sign => %03d | 2 integers for minutes, 1 for point, 4 for minutes decimals
    sprintf(buf, "%03d%07.4f", d, minutes);
  }
}

/**
 * Get Date from system
 */
void gps_nmea_get_system_date_str(char *buf_date, size_t buf_date_size, char *buf_time, size_t buf_time_size)
{
  // days from 1970-01-01
  const int64_t gps_epoch_days = 3657;
  // Total time gps
  uint64_t gps_sec = (uint64_t)gps.week * 7 * 24 * 3600 + gps.tow / 1000;
  // Day gps + gps epoch
  int64_t days = gps_epoch_days + gps_sec / 86400;

  // Algo to get year month day from Week and tow
  // Shift day count so that day 0 corresponds to the Gregorian epoch (0000-03-01)
  days += 719468;
  // Determine the 400-year Gregorian cycle, One Gregorian era = 400 years = 146097 days.
  int64_t era = (days >= 0 ? days : days - 146096) / 146097;
  // Day Of Era: number of days elapsed within the current 400-year cycle.
  uint64_t doe = (uint64_t)(days - era * 146097);
  // Year Of Era: compute the year number inside the cycle, correcting for leap years (every 4 years, except centuries, except 400-year multiples).
  uint64_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  // Convert "year in era" to an absolute Gregorian year.
  int64_t y = (int64_t)yoe + era * 400;
  // Day Of Year: subtract all full days from completed years to get day index within the year.
  uint64_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  // Month part: day-of-year into a month index in a calendar where March = 0 and February = 11
  uint64_t mp = (5 * doy + 2) / 153;
  // Compute the day-of-month
  uint64_t d = doy - (153 * mp + 2) / 5 + 1;
  // Convert shifted month index back to standard month  0-12
  uint64_t m = mp + (mp < 10 ? 3 : -9);
  y += (m <= 2);

  // Hours / minutes / secondes
  uint32_t sec_of_day = gps_sec % 86400;
  uint8_t hour = sec_of_day / 3600;
  uint8_t min  = (sec_of_day % 3600) / 60;
  uint8_t sec  = sec_of_day % 60;

  // Format : DDMMYY
  snprintf(buf_date, buf_date_size, "%02d%02d%02d",
           (uint8_t)d,
           (uint8_t)m,
           (uint8_t)(y % 100));

  // Format : HHMMSS
  snprintf(buf_time, buf_time_size, "%02d%02d%02d",
           hour,
           min,
           sec);
}

/**
 * Function to send payload to UART
 */
void nmea_send(const char *payload, int payload_length)
{
  uart_put_buffer(&NMEA_SEND_UART, 0, (const uint8_t *)payload, payload_length);
}

