/*
 *  
 * Copyright (C) 2008 Marcus Wolschon
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
 * file gps_nmea.c
 * brief Parser for the NMEA protocol
 *
 * This file is a drop-in replacement for gps_ubx.c
 *
 * TODO: THIS NMEA-PARSER IS NOT WELL TESTED AND INCOMPLETE!!!
 * Status:
 *  Parsing GGA and RMC is complete, GSA and other records are
 *  incomplete.
 */

#include <inttypes.h>
#include <string.h> 
#include <math.h>
#include <stdlib.h>
#ifdef DEBUG_NMEA
// do debug-output if run on the DEBUG_NMEA-target

#endif

#include "generated/airframe.h"
#include "autopilot.h"
#include "generated/flight_plan.h"
#include "mcu_periph/uart.h"
#include "gps.h"
#include "gps_nmea.h"
#include "subsystems/nav.h"
#include "latlong.h"

int32_t  gps_lat;  // latitude in degrees * 1e-7
int32_t  gps_lon;  // longitude in degrees * 1e-7
uint16_t gps_PDOP; //precision
bool_t   gps_pos_available = FALSE;

uint16_t gps_week;
uint32_t gps_itow;
int32_t  gps_alt;
uint16_t gps_gspeed; // in cm/s
int16_t  gps_climb;
int16_t  gps_course;
int32_t  gps_utm_east, gps_utm_north;
uint8_t  gps_utm_zone;
uint8_t  gps_mode;

#ifdef GPS_CONFIGURE
static uint8_t gps_status_config;
#endif

//AD added gps_configuring variable
//   in gps.h if gps_configuring is true GpsParseOrConfigure() checks value of variable 
//   and calls gps_configure() or parse_gps_msg() depending on its value

bool_t gps_configuring;

// true if parse_ubx() has a complete message and parse_gps_msg() shall parse it
volatile bool_t gps_msg_received = FALSE;

uint8_t  ubx_id, ubx_class; // unused
uint16_t gps_reset;         // unused

uint32_t gps_Pacc, gps_Sacc;
uint8_t gps_numSV;         // number of satelites in view

struct svinfo gps_svinfos[GPS_NB_CHANNELS];
uint8_t gps_nb_channels;
uint8_t gps_nb_ovrn;        // number if incomplete nmea-messages





////////////////////////////////////////////////////////
//       uart-configuration

//AD
// added GPS_CONFIG_INIT define equal to 0
#define GPS_CONFIG_INIT 0
void gps_init( void ) {
#ifdef GPS_CONFIGURE
  gps_status_config = GPS_CONFIG_INIT;
  gps_configuring = TRUE;
#endif
}



#ifdef GPS_CONFIGURE
/* GPS dynamic configuration */

#include "uart.h"

void gps_configure_uart ( void ) {
  //UbxSend_CFG_PRT(0x01, 0x0, 0x0, 0x000008D0, GPS_BAUD, UBX_PROTO_MASK, UBX_PROTO_MASK, 0x0, 0x0);  
  //while (GpsUartRunning) ; /* FIXME */
  GpsUartInitParam( UART_BAUD(GPS_BAUD),  UART_8N1, UART_FIFO_8);
}

void gps_configure ( void ) {
   gps_configuring=FALSE;	
}
#endif /* GPS_CONFIGURE */

////////////////////////////////////////////////////////
//       nmea-parser


/**
 * The buffer, we store one nmea-line in
 * for parsing.
 */
#define NMEA_MAXLEN 255
char nmea_msg_buf[NMEA_MAXLEN];
int  nmea_msg_len = 0;
/*
int GpsFixValid() {
   return gps_pos_available;
} 
*/
/**
 * parse GPGSA-nmea-messages stored in
 * nmea_msg_buf .
 */
void parse_nmea_GPGSA() {
      int i = 8;     // current position in the message
//      char* endptr;  // end of parsed substrings

      // attempt to reject empty packets right away
      if(nmea_msg_buf[i]==',' && nmea_msg_buf[i+1]==',') {
            NMEA_PRINT("p_GPGSA() - skipping empty message\n\r");
            return;
      }

      // get auto2D/3D
      // ignored
      while(nmea_msg_buf[i++] != ',') {              // next field: fix
         if (i >= nmea_msg_len) {
	   NMEA_PRINT("p_GPGSA() - skipping incomplete message\n\r");
           return;
         }
     }

      // get 2D/3D-fix
      // set gps_mode=3=3d, 2=2d, 1=no fix or 0
      gps_mode = atoi(&nmea_msg_buf[i]);
      if (gps_mode == 1)
         gps_mode = 0;
      NMEA_PRINT("p_GPGSA() - gps_mode=%i (3=3D)\n\r", gps_mode);
      while(nmea_msg_buf[i++] != ',') {              // next field:sateline-number-0
         if (i >= nmea_msg_len) {
           NMEA_PRINT("p_GPGSA() - skipping incomplete message\n\r");
           return;
          }
     }

      //int satcount = 0;

      // TODO: get sateline-numbers for gps_svinfos
}

/**
 * parse GPRMC-nmea-messages stored in
 * nmea_msg_buf .
 */
void parse_nmea_GPRMC() {
      int i = 8;     // current position in the message
      char* endptr;  // end of parsed substrings

      // attempt to reject empty packets right away
      if(nmea_msg_buf[i]==',' && nmea_msg_buf[i+1]==',') {
            NMEA_PRINT("p_GPRMC() - skipping empty message\n\r");
           return;
      }

      // get time
      // ignored
      while(nmea_msg_buf[i++] != ',') {              // next field: warning
         if (i >= nmea_msg_len) {
           NMEA_PRINT("p_GPRMC() - skipping incomplete message\n\r");
           return;
	   }
     }

      // get warning
      // ignored
      while(nmea_msg_buf[i++] != ',') {              // next field: lat
         if (i >= nmea_msg_len) {
           NMEA_PRINT("p_GPRMC() - skipping incomplete message\n\r");
           return;
	   }
     }
      // get lat
      // ignored
      while(nmea_msg_buf[i++] != ',') {              // next field: N/S
         if (i >= nmea_msg_len) {
           NMEA_PRINT("p_GPRMC() - skipping incomplete message\n\r");
           return;
	   }
     }
      // get North/South
      // ignored
      while(nmea_msg_buf[i++] != ',') {              // next field: lon
         if (i >= nmea_msg_len) {
           NMEA_PRINT("p_GPRMC() - skipping incomplete message\n\r");
           return;
	   }
     }
      // get lon
      // ignored
      while(nmea_msg_buf[i++] != ',') {              // next field: E/W
         if (i >= nmea_msg_len) {
           NMEA_PRINT("p_GPRMC() - skipping incomplete message\n\r");
           return;
	   }
     }
      // get eath/west
      // ignored
      while(nmea_msg_buf[i++] != ',') {              // next field: speed
         if (i >= nmea_msg_len) {
           NMEA_PRINT("p_GPRMC() - skipping incomplete message\n\r");
           return;
	   }
     }
      // get speed
      double speed = strtod(&nmea_msg_buf[i], &endptr);
      gps_gspeed = speed * 1.852 * 100 / (60*60);
      NMEA_PRINT("p_GPRMC() - ground-speed=%d knot = %d cm/s\n\r", (speed*1000), (gps_gspeed*1000));
      while(nmea_msg_buf[i++] != ',') {              // next field: course
         if (i >= nmea_msg_len) {
           NMEA_PRINT("p_GPRMC() - skipping incomplete message\n\r");
           return;
	   }
     }
      double course = strtod(&nmea_msg_buf[i], &endptr);
      gps_course=course*10;
      NMEA_PRINT("COURSE: %d \n\r",gps_course);
}


/**
 * parse GPGGA-nmea-messages stored in
 * nmea_msg_buf .
 */
void parse_nmea_GPGGA() {
      int i = 8;     // current position in the message
      char* endptr;  // end of parsed substrings
      double degrees, minutesfrac;

      // attempt to reject empty packets right away
      if(nmea_msg_buf[i]==',' && nmea_msg_buf[i+1]==',') {
            NMEA_PRINT("p_GPGGA() - skipping empty message\n\r");
            return;
      }

      // get UTC time [hhmmss.sss]
      // ignored GpsInfo.PosLLA.TimeOfFix.f = strtod(&packet[i], &endptr);
      double time = strtod(&nmea_msg_buf[i],&endptr);
      gps_itow = (uint32_t)((time+1)*1000);

//AD TODO: strtod itow
      while(nmea_msg_buf[i++] != ',') {              // next field: latitude
         if (i >= nmea_msg_len) {
           NMEA_PRINT("p_GPGGA() - skipping incomplete message\n\r");
	   return;
         }
      }

      // get latitude [ddmm.mmmmm]
      double lat = strtod(&nmea_msg_buf[i], &endptr);
      // convert to pure degrees [dd.dddd] format
      minutesfrac = modf(lat/100, &degrees);
      lat = degrees + (minutesfrac*100)/60;
      // convert to radians
      //GpsInfo.PosLLA.lat.f *= (M_PI/180);
      while(nmea_msg_buf[i++] != ',') {              // next field: N/S indicator
         if (i >= nmea_msg_len) {
           NMEA_PRINT("p_GPGGA() - skipping incomplete message\n\r");
	   return;
	 }
      }
     
      // correct latitute for N/S
      if(nmea_msg_buf[i] == 'S')
         lat = -lat;
      while(nmea_msg_buf[i++] != ',') {              // next field: longitude
         if (i >= nmea_msg_len)
	   return;
      }

      gps_lat = lat * 1e7; // convert to fixed-point
      NMEA_PRINT("p_GPGGA() - lat=%d gps_lat=%i\n\r", (lat*1000), gps_lat);
     
      // get longitude [ddmm.mmmmm]
      double lon = strtod(&nmea_msg_buf[i], &endptr);
      // convert to pure degrees [dd.dddd] format
      minutesfrac = modf(lon/100, &degrees);
      lon = degrees + (minutesfrac*100)/60;
      // convert to radians
      //GpsInfo.PosLLA.lon.f *= (M_PI/180);
      while(nmea_msg_buf[i++] != ',') {              // next field: E/W indicator
         if (i >= nmea_msg_len)
	   return;
      }
 
      // correct latitute for E/W
      if(nmea_msg_buf[i] == 'W')
         lon = -lon;
      while(nmea_msg_buf[i++] != ',') {              // next field: position fix status
         if (i >= nmea_msg_len)
	   return;
      }
 
      gps_lon = lon * 1e7; // convert to fixed-point
      NMEA_PRINT("p_GPGGA() - lon=%d gps_lon=%i time=%u\n\r", (lon*1000), gps_lon,gps_itow);

      latlong_utm_of(RadOfDeg(lat), RadOfDeg(lon), nav_utm_zone0);

      gps_utm_east = latlong_utm_x * 100;
      gps_utm_north = latlong_utm_y * 100;
      gps_utm_zone = nav_utm_zone0;


      // position fix status
      // 0 = Invalid, 1 = Valid SPS, 2 = Valid DGPS, 3 = Valid PPS
      // check for good position fix
      if( (nmea_msg_buf[i] != '0') && (nmea_msg_buf[i] != ',') )  {
        gps_pos_available = TRUE;
        NMEA_PRINT("p_GPGGA() - POS_AVAILABLE == TRUE\n\r");
      } else {
        gps_pos_available = FALSE;
        NMEA_PRINT("p_GPGGA() - gps_pos_available == false\n\r");
      }
      while(nmea_msg_buf[i++] != ',') {              // next field: satellites used
         if (i >= nmea_msg_len) {
           NMEA_PRINT("p_GPGGA() - skipping incomplete message\n\r\r");
	   return;
	 }
      }
     
      // get number of satellites used in GPS solution
      gps_numSV = atoi(&nmea_msg_buf[i]);
           NMEA_PRINT("p_GPGGA() - gps_numSatlitesUsed=%i\n\r", gps_numSV);
      while(nmea_msg_buf[i++] != ',') {              // next field: HDOP (horizontal dilution of precision)
         if (i >= nmea_msg_len) {
           NMEA_PRINT("p_GPGGA() - skipping incomplete message\n\r");
	   return;
	 }
      }
      while(nmea_msg_buf[i++] != ',') {              // next field: altitude
         if (i >= nmea_msg_len) {
           NMEA_PRINT("p_GPGGA() - skipping incomplete message\n\r");
	   return;
	 }
      }
     
      // get altitude (in meters)
      double alt = strtod(&nmea_msg_buf[i], &endptr);
      gps_alt = alt * 10;
      NMEA_PRINT("p_GPGGA() - gps_alt=%i\n\r", gps_alt);

      while(nmea_msg_buf[i++] != ',') {              // next field: altitude units, always 'M'
         if (i >= nmea_msg_len)
	   return;
      }
      while(nmea_msg_buf[i++] != ',') {              // next field: geoid seperation
         if (i >= nmea_msg_len)
	   return;
      }
      while(nmea_msg_buf[i++] != ',') {              // next field: seperation units
         if (i >= nmea_msg_len)
	   return;
      }
      while(nmea_msg_buf[i++] != ',') {              // next field: DGPS age
         if (i >= nmea_msg_len)
	   return;
      }
      while(nmea_msg_buf[i++] != ',') {              // next field: DGPS station ID
         if (i >= nmea_msg_len)
	   return;
      }
      //while(nmea_msg_buf[i++] != '*');              // next field: checksum
}

/**
 * parse_nmea_char() has a complete line.
 * Find out what type of message it is and
 * hand it to the parser for that type.
 */
void parse_gps_msg( void ) {

      if(nmea_msg_len > 5 && !strncmp(nmea_msg_buf , "GPRMC", 5)) {
         nmea_msg_buf[nmea_msg_len] = 0;
         NMEA_PRINT("parsing RMC: \"%s\" \n\r",nmea_msg_buf);
	 NMEA_PRINT("RMC");
         parse_nmea_GPRMC();
      } else
      if(nmea_msg_len > 5 && !strncmp(nmea_msg_buf , "GPGGA", 5)) {
         nmea_msg_buf[nmea_msg_len] = 0;
         NMEA_PRINT("parse_gps_msg() - parsing GGA gps-message \"%s\" \n\r",nmea_msg_buf);
	 NMEA_PRINT("GGA");
         parse_nmea_GPGGA();
      } else
      if(nmea_msg_len > 5 && !strncmp(nmea_msg_buf , "GPGSA", 5)) {
         nmea_msg_buf[nmea_msg_len] = 0;
         NMEA_PRINT("GSA: \"%s\" \n\r",nmea_msg_buf);
	 NMEA_PRINT("GSA");
         parse_nmea_GPGSA();
      } else {
         nmea_msg_buf[nmea_msg_len] = 0;
         NMEA_PRINT("ignoring: len=%i \n\r \"%s\" \n\r", nmea_msg_len, nmea_msg_buf);
      }

      // reset message-buffer
      nmea_msg_len = 0;
}


/**
 * This is the actual parser.
 * It reads one character at a time
 * setting gps_msg_received to TRUE
 * after a full line.
 */
void parse_nmea_char( uint8_t c ) {
  //reject empty lines
  if (nmea_msg_len == 0) {
     if (c == '\r' || c == '\n' || c == '$')
       return;
  }

  // fill the buffer, unless it's full
  if (nmea_msg_len < NMEA_MAXLEN - 1) {

      // messages end with a linefeed
	  //AD: TRUNK:       if (c == '\r' || c == '\n') 
      if (c == '\r' || c == '\n') {
        gps_msg_received = TRUE;
      } else {
        nmea_msg_buf[nmea_msg_len] = c;
        nmea_msg_len ++;
      }
  }

  if (nmea_msg_len >= NMEA_MAXLEN - 1)
     gps_msg_received = TRUE;
}
