/*
 * Copyright (C) Fabien Bonneval
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
 * @file "modules/e_identification_fr/e_identification_fr.c"
 * @author Fabien Bonneval
 * Format and send via UART tracking data according to French requirements.
 */

#include "modules/e_identification_fr/e_identification_fr.h"
#include "state.h"
#include "mcu_periph/uart.h"
#include "subsystems/gps.h"
#include "generated/airframe.h"
#include "generated/flight_plan.h"

#if defined(ROTORCRAFT_FIRMWARE)
#include "subsystems/navigation/waypoints.h"
#endif

#define MAX_BUF_LEN 74

#define LEN_ID_FR 30
#define LEN_ID_SERIAL 24

struct uart_periph *dev;
char id_fr[LEN_ID_FR];

static int put_ID(uint8_t* buf) {
  buf[0] = E_ID_ID_FR;
  buf[1] = strlen(id_fr);
  memcpy(buf+2, id_fr, LEN_ID_FR);
  return LEN_ID_FR + 2;
}

static int put_lat(uint8_t* buf) {
  int32_t lat = stateGetPositionLla_i()->lat / 1e2;
  buf[0] = E_ID_LAT;
  buf[1] = sizeof(lat);
  memcpy(buf+2, &lat, sizeof(lat));
  return sizeof(lat)+2;
}

static int put_lon(uint8_t* buf) {
  int32_t lon = stateGetPositionLla_i()->lon / 1e2;
  buf[0] = E_ID_LON;
  buf[1] = sizeof(lon);
  memcpy(buf+2, &lon, sizeof(lon));
  return sizeof(lon)+2;
}

static int put_alt(uint8_t* buf) {
  int16_t alt = gps.hmsl / 1e3;
  buf[0] = E_ID_HMSL;
  buf[1] = sizeof(alt);
  memcpy(buf+2, &alt, sizeof(alt));
  return sizeof(alt)+2;
}

static int put_horizontal_speed(uint8_t* buf) {
  uint8_t speed = (uint8_t) stateGetHorizontalSpeedNorm_f();
  buf[0] = E_ID_H_SPEED;
  buf[1] = sizeof(speed);
  memcpy(buf+2, &speed, sizeof(speed));
  return sizeof(speed)+2;
}

static int put_route(uint8_t* buf) {
  int32_t route_normalized = INT32_DEG_OF_RAD(stateGetHorizontalSpeedDir_f());
  uint16_t route;
  if(route_normalized >=0) {
    route = (uint16_t) route_normalized;
  } else {
    route = (uint16_t) (360 + route_normalized);
  }
  buf[0] = E_ID_ROUTE;
  buf[1] = sizeof(route);
  memcpy(buf+2, &route, sizeof(route));
  return sizeof(route)+2;
}

static int put_lat_lon_home(uint8_t* buf) {
  struct LlaCoor_i* ptr_home_lla_i;
#if defined(FIXEDWING_FIRMWARE)
  struct LlaCoor_i home_lla_i;
  struct LlaCoor_f home_lla_f;
  struct UtmCoor_f home_utm_f;
  
  home_utm_f.east = nav_utm_east0 +  waypoints[WP_HOME].x;
  home_utm_f.north = nav_utm_north0 + waypoints[WP_HOME].y;
  home_utm_f.zone = nav_utm_zone0;
  home_utm_f.alt = 0;    // don't care about the altitude
  
  lla_of_utm_f(&home_lla_f, &home_utm_f);
  LLA_BFP_OF_REAL(home_lla_i, home_lla_f);
  ptr_home_lla_i = &home_lla_i;


#elif defined(ROTORCRAFT_FIRMWARE)
  ptr_home_lla_i = waypoint_get_lla(WP_HOME);
#else
    #error  Not a fixedwing or a rotorcraft, not yet supported
#endif
  
  int32_t lat_home = ptr_home_lla_i->lat / 1e2;
  int32_t lon_home = ptr_home_lla_i->lon / 1e2;
  
  int offset = 0;
  buf[offset++] = E_ID_LAT_TO;
  buf[offset++] = sizeof(lat_home);
  memcpy(buf+offset, &lat_home, sizeof(lat_home));
  offset += sizeof(lat_home);
  
  buf[offset++] = E_ID_LON_TO;
  buf[offset++] = sizeof(lon_home);
  memcpy(buf+offset, &lon_home, sizeof(lon_home));
  offset += sizeof(lon_home);
  
  return offset;
}

void e_identification_fr_init() {
  dev = &(E_ID_DEV);
  for(int i=0; i<LEN_ID_FR; i++) {
    id_fr[i] = '0';
  }
  
  memcpy(id_fr, ID_MANUFACTURER, 3);
  memcpy(id_fr+3, ID_MODEL, 3);
  int id_len = Min(strlen(ID_SERIAL), LEN_ID_SERIAL);
  memcpy(id_fr+LEN_ID_SERIAL+6-id_len, ID_SERIAL, id_len);
}


void e_identification_fr_periodic() {
  
  if(gps.fix) {
    uint8_t buf[MAX_BUF_LEN];
    buf[0] = 0X99;  //PPRZ_STX
    buf[1] = 0;   //filled later with message length
    uint8_t offset = 2;
    
    offset += put_ID (buf + offset);
    offset += put_lat(buf + offset);
    offset += put_lon(buf + offset);
    offset += put_alt(buf + offset);
    offset += put_lat_lon_home(buf + offset);
    offset += put_route(buf + offset);
    offset += put_horizontal_speed(buf + offset);
    
    buf[1] = offset + 2;  //include from STX to checksum_b
    
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;
    for(int i=1; i<offset; i++) {
      ck_a += buf[i];
      ck_b += ck_a;
    }
    buf[offset++] = ck_a;
    buf[offset++] = ck_b;
    
    uart_put_buffer(dev, 0, buf, offset);
  }
}

