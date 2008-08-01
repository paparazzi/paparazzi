/*
 * Paparazzi mcu0 $Id$
 *  
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
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

/** \file xsens.c
 * \brief Parser for the Xsens protocol
 */

#include "quad_ins.h"

#include <inttypes.h>

#include "airframe.h"
#include "led.h"
#include "booz_estimator.h"
#include "quad_gps.h"
#include "quad_vfilter.h"
#include "agl_vfilter.h"
#include "booz_inter_mcu.h"
#include "booz_autopilot.h"

#include "downlink.h"
#include "messages.h"

#include "latlong.h"

/* comes from booz_inter_mcu.h */
struct booz_inter_mcu_state inter_mcu_state;

volatile uint8_t ins_msg_received;

#define XsensInitCheksum() { send_ck = 0; }
#define XsensUpdateChecksum(c) { send_ck += c; }

#define XsensSend1(c) { uint8_t i8=c; InsUartSend1(i8); XsensUpdateChecksum(i8); }
#define XsensSend1ByAddr(x) { XsensSend1(*x); }
#define XsensSend2ByAddr(x) { XsensSend1(*(x+1)); XsensSend1(*x); }
#define XsensSend4ByAddr(x) { XsensSend1(*(x+3)); XsensSend1(*(x+2)); XsensSend1(*(x+1)); XsensSend1(*x); }

#define XsensHeader(msg_id, len) { \
  InsUartSend1(XSENS_START); \
  XsensInitCheksum(); \
  XsensSend1(XSENS_BID); \
  XsensSend1(msg_id); \
  XsensSend1(len); \
}
#define XsensTrailer() { uint8_t i8=0x100-send_ck; InsUartSend1(i8); }

/** Includes macros generated from xsens_MTi-G.xml */ 
#include "xsens_protocol.h"

 
#define XSENS_MAX_PAYLOAD 254
uint8_t xsens_msg_buf[XSENS_MAX_PAYLOAD];

/* output mode : calibrated, orientation, position, velocity, status */
#ifndef XSENS_OUTPUT_MODE
#define XSENS_OUTPUT_MODE 0x0836
#endif
/* output settings : timestamp, euler, acc, rate, mag, float, no aux, lla, m/s, NED */
#ifndef XSENS_OUTPUT_SETTINGS
#define XSENS_OUTPUT_SETTINGS 0x80000C05
#endif

#ifndef XSENS_NED
#define XSENS_NED 1
#endif

#define UNINIT        0
#define GOT_START     1
#define GOT_BID       2
#define GOT_MID       3
#define GOT_LEN       4
#define GOT_DATA      5
#define GOT_CHECKSUM  6

uint8_t xsens_errorcode;
uint8_t xsens_msg_status;
uint16_t xsens_time_stamp;
uint16_t xsens_output_mode;
uint32_t xsens_output_settings;

static uint8_t xsens_id;
static uint8_t xsens_status;
static uint8_t xsens_len;
static uint8_t xsens_msg_idx;
static uint8_t ck;
uint8_t send_ck;

void quad_ins_init( void ) {

//#ifdef NO_INTERNAL_GPS
//  // disconnect the internal GPS
//  Configure_GPS_RESET_Pin();
//  Set_GPS_RESET_Pin_LOW();
//#endif

  xsens_status = UNINIT;

  xsens_msg_status = 0;
  xsens_time_stamp = 0;
  xsens_output_mode = XSENS_OUTPUT_MODE;
  xsens_output_settings = XSENS_OUTPUT_SETTINGS;
  /* send mode and settings to MT */
  XSENS_GoToConfig();
  XSENS_SetOutputMode(xsens_output_mode);
  XSENS_SetOutputSettings(xsens_output_settings);
  //XSENS_GoToMeasurment();
}

void quad_ins_periodic_task( void ) {
  static uint8_t _1hz = 0;
  _1hz++;
  if (_1hz >= 100) _1hz = 0;
  switch (_1hz) {
    case 0:
      //XSENS_GoToConfig();
      XSENS_ReqGPSStatus();
      //XSENS_GoToMeasurment();
      break;
  }
}

void parse_ins_msg( void ) {
    uint8_t offset = 0;
  if (xsens_id == XSENS_ReqOutputModeAck_ID) {
    xsens_output_mode = XSENS_ReqOutputModeAck_mode(xsens_msg_buf);
  }
  else if (xsens_id == XSENS_ReqOutputSettings_ID) {
    xsens_output_settings = XSENS_ReqOutputSettingsAck_settings(xsens_msg_buf);
  }
  else if (xsens_id == XSENS_Error_ID) {
    xsens_errorcode = XSENS_Error_errorcode(xsens_msg_buf);
  }
  else if (xsens_id == XSENS_GPSStatus_ID) {
    //DOWNLINK_SEND_DEBUG(xsens_len,xsens_msg_buf);
    gps_nb_channels = XSENS_GPSStatus_nch(xsens_msg_buf);
    uint8_t i;
    for(i = 0; i < Min(gps_nb_channels, GPS_NB_CHANNELS); i++) {
      uint8_t ch = XSENS_GPSStatus_chn(xsens_msg_buf,i);
      if (ch > GPS_NB_CHANNELS) continue;
      gps_svinfos[ch].svid = XSENS_GPSStatus_svid(xsens_msg_buf, i);
      gps_svinfos[ch].flags = XSENS_GPSStatus_bitmask(xsens_msg_buf, i);
      gps_svinfos[ch].qi = XSENS_GPSStatus_qi(xsens_msg_buf, i);
      gps_svinfos[ch].cno = XSENS_GPSStatus_cnr(xsens_msg_buf, i);
      //DOWNLINK_SEND_SVINFO(&ch, &gps_svinfos[ch].svid, &gps_svinfos[ch].flags, &gps_svinfos[ch].qi, &gps_svinfos[ch].cno,&gps_svinfos[ch].elev, &gps_svinfos[ch].azim);
    }
  }
  else if (xsens_id == XSENS_MTData_ID) {
    //uint8_t offset = 0;
    /* test RAW modes else calibrated modes */
    //if ((XSENS_MASK_RAWInertial(xsens_output_mode)) || (XSENS_MASK_RAWGPS(xsens_output_mode))) {
      if (XSENS_MASK_RAWInertial(xsens_output_mode)) {
        booz_estimator_uf_p = XSENS_DATA_RAWInertial_gyrX(xsens_msg_buf,offset);
        booz_estimator_p = booz_estimator_uf_p;
        booz_estimator_uf_q = XSENS_DATA_RAWInertial_gyrY(xsens_msg_buf,offset);
        booz_estimator_q = booz_estimator_uf_q;
        booz_estimator_uf_r = XSENS_DATA_RAWInertial_gyrZ(xsens_msg_buf,offset);
        booz_estimator_r = booz_estimator_uf_r;
        //booz_autopilot_mode = BOOZ_AP_MODE_RATE;
        offset += XSENS_DATA_RAWInertial_LENGTH;
      }
      if (XSENS_MASK_RAWGPS(xsens_output_mode)) {
        gps_itow = XSENS_DATA_RAWGPS_itow(xsens_msg_buf,offset);
        gps_lat = XSENS_DATA_RAWGPS_lat(xsens_msg_buf,offset);
        gps_lon = XSENS_DATA_RAWGPS_lon(xsens_msg_buf,offset);
#ifdef NAV_HORIZONTAL
        /* Set the real UTM zone */
        gps_utm_zone = (gps_lon/1e7+180) / 6 + 1;
        latlong_utm_of(RadOfDeg(gps_lat/1e7), RadOfDeg(gps_lon/1e7), gps_utm_zone);
        /* utm */
        gps_utm_east = latlong_utm_x * 100;
        gps_utm_north = latlong_utm_y * 100;
        booz_estimator_x = latlong_utm_x;
        booz_estimator_y = latlong_utm_y;
#endif
        gps_alt = XSENS_DATA_RAWGPS_alt(xsens_msg_buf,offset) / 10;
        booz_estimator_z = -(float)gps_alt / 100.;
        booz_estimator_vx = (float)XSENS_DATA_RAWGPS_vel_e(xsens_msg_buf,offset) / 100.;
        booz_estimator_vy = (float)XSENS_DATA_RAWGPS_vel_n(xsens_msg_buf,offset) / 100.;
        booz_estimator_vz = (float)XSENS_DATA_RAWGPS_vel_d(xsens_msg_buf,offset) / 100.;
        gps_climb = -XSENS_DATA_RAWGPS_vel_d(xsens_msg_buf,offset) / 10;
        gps_Pacc = XSENS_DATA_RAWGPS_hacc(xsens_msg_buf,offset);
        gps_Sacc = XSENS_DATA_RAWGPS_sacc(xsens_msg_buf,offset);
        offset += XSENS_DATA_RAWGPS_LENGTH;
      }
    //} else {
      if (XSENS_MASK_Temp(xsens_output_mode)) {
        offset += XSENS_DATA_Temp_LENGTH;
      }
      if (XSENS_MASK_Calibrated(xsens_output_mode)) {
        uint8_t l = 0;
        if (!XSENS_MASK_AccOut(xsens_output_settings)) {
          inter_mcu_state.accel[AXIS_X] = XSENS_DATA_Calibrated_accX(xsens_msg_buf,offset);
          inter_mcu_state.accel[AXIS_Y] = XSENS_NED * XSENS_DATA_Calibrated_accY(xsens_msg_buf,offset);
          inter_mcu_state.accel[AXIS_Z] = XSENS_NED * XSENS_DATA_Calibrated_accZ(xsens_msg_buf,offset);
#ifdef USE_VFILTER
#ifdef USE_BARO_MS5534A
          inter_mcu_state.pos[AXIS_Z] = quad_vf_z;
          inter_mcu_state.speed[AXIS_Z] = quad_vf_zdot;
          gps_alt = inter_mcu_state.pos[AXIS_Z] * 100;
          gps_climb = (int16_t)(inter_mcu_state.speed[AXIS_Z] * 100);
#endif
#ifdef USE_TELEMETER
          booz_estimator_agl_z = agl_vf_z;
          booz_estimator_agl_zdot = agl_vf_zdot;
#endif
#endif
          l++;
        }
        if (!XSENS_MASK_GyrOut(xsens_output_settings)) {
          inter_mcu_state.r_rates[AXIS_P] = XSENS_DATA_Calibrated_gyrX(xsens_msg_buf,offset) * RATE_PI_S/M_PI;
          inter_mcu_state.r_rates[AXIS_Q] = XSENS_NED * XSENS_DATA_Calibrated_gyrY(xsens_msg_buf,offset) * RATE_PI_S/M_PI;
          inter_mcu_state.r_rates[AXIS_R] = XSENS_NED * XSENS_DATA_Calibrated_gyrZ(xsens_msg_buf,offset) * RATE_PI_S/M_PI;
          inter_mcu_state.f_rates[AXIS_P] = inter_mcu_state.r_rates[AXIS_P];
          inter_mcu_state.f_rates[AXIS_Q] = inter_mcu_state.r_rates[AXIS_Q];
          inter_mcu_state.f_rates[AXIS_R] = inter_mcu_state.r_rates[AXIS_R];
          l++;
        }
        if (!XSENS_MASK_MagOut(xsens_output_settings)) {
          l++;
        }
        offset += l * XSENS_DATA_Calibrated_LENGTH / 3;
      }
      if (XSENS_MASK_Orientation(xsens_output_mode)) {
        if (XSENS_MASK_OrientationMode(xsens_output_settings) == 0x00) {
          float q0 = XSENS_DATA_Quaternion_q0(xsens_msg_buf,offset);
          float q1 = XSENS_DATA_Quaternion_q1(xsens_msg_buf,offset);
          float q2 = XSENS_DATA_Quaternion_q2(xsens_msg_buf,offset);
          float q3 = XSENS_DATA_Quaternion_q3(xsens_msg_buf,offset);
          float dcm00 = 1.0 - 2 * (q2*q2 + q3*q3);
          float dcm01 =       2 * (q1*q2 + q0*q3);
          float dcm02 =       2 * (q1*q3 - q0*q2);
          float dcm12 =       2 * (q2*q3 + q0*q1);
          float dcm22 = 1.0 - 2 * (q1*q1 + q2*q2);
          inter_mcu_state.f_eulers[AXIS_X] = atan2(dcm12, dcm22) * ANGLE_PI/M_PI;
          inter_mcu_state.f_eulers[AXIS_Y] = -asin(dcm02)        * ANGLE_PI/M_PI;
          inter_mcu_state.f_eulers[AXIS_Z] = atan2(dcm01, dcm00) * ANGLE_PI/M_PI;
          gps_course = inter_mcu_state.f_eulers[AXIS_Z] * 1800 / ANGLE_PI;
          offset += XSENS_DATA_Quaternion_LENGTH;
        }
        if (XSENS_MASK_OrientationMode(xsens_output_settings) == 0x01) {
          inter_mcu_state.f_eulers[AXIS_X] = XSENS_DATA_Euler_roll(xsens_msg_buf,offset)  * ANGLE_PI/180;
          inter_mcu_state.f_eulers[AXIS_Y] = XSENS_NED * XSENS_DATA_Euler_pitch(xsens_msg_buf,offset) * ANGLE_PI/180;
          inter_mcu_state.f_eulers[AXIS_Z] = XSENS_NED * XSENS_DATA_Euler_yaw(xsens_msg_buf,offset)   * ANGLE_PI/180;
          gps_course = (int16_t)(XSENS_NED * XSENS_DATA_Euler_yaw(xsens_msg_buf,offset) * 10);
          offset += XSENS_DATA_Euler_LENGTH;
        }
        if (XSENS_MASK_OrientationMode(xsens_output_settings) == 0x10) {
          offset += XSENS_DATA_Matrix_LENGTH;
        }
      }
      if (XSENS_MASK_Auxiliary(xsens_output_mode)) {
        uint8_t l = 0;
        if (!XSENS_MASK_Aux1Out(xsens_output_settings)) {
          l++;
        }
        if (!XSENS_MASK_Aux2Out(xsens_output_settings)) {
          l++;
        }
        offset += l * XSENS_DATA_Auxiliary_LENGTH / 2;
      }
      if (XSENS_MASK_Position(xsens_output_mode)) {
    //DOWNLINK_SEND_DEBUG(xsens_len,xsens_msg_buf);
        float lat = XSENS_DATA_Position_lat(xsens_msg_buf,offset);
        float lon = XSENS_DATA_Position_lon(xsens_msg_buf,offset);
        gps_lat = (int32_t)(lat * 1e7);
        gps_lon = (int32_t)(lon * 1e7);
//#ifdef NAV_HORIZONTAL
        gps_utm_zone = (lon+180) / 6 + 1;
        latlong_utm_of(RadOfDeg(lat), RadOfDeg(lon), gps_utm_zone);
//#endif
        inter_mcu_state.pos[AXIS_X] = latlong_utm_x;
        inter_mcu_state.pos[AXIS_Y] = latlong_utm_y;
        gps_utm_east  = inter_mcu_state.pos[AXIS_X] * 100;
        gps_utm_north = inter_mcu_state.pos[AXIS_Y] * 100;
#ifndef USE_VFILTER
        inter_mcu_state.pos[AXIS_Z] = -XSENS_DATA_Position_alt(xsens_msg_buf,offset);
        gps_alt = inter_mcu_state.pos[AXIS_Z] * 100;
#endif
        offset += XSENS_DATA_Position_LENGTH;
      }
      if (XSENS_MASK_Velocity(xsens_output_mode)) {
    //DOWNLINK_SEND_DEBUG(xsens_len,xsens_msg_buf);
        inter_mcu_state.speed[AXIS_X] = XSENS_DATA_Velocity_vx(xsens_msg_buf,offset);
        inter_mcu_state.speed[AXIS_Y] = XSENS_NED * XSENS_DATA_Velocity_vy(xsens_msg_buf,offset);
#ifndef USE_VFILTER
        inter_mcu_state.speed[AXIS_Z] = XSENS_NED * XSENS_DATA_Velocity_vz(xsens_msg_buf,offset);
        gps_climb = (int16_t)(-inter_mcu_state.speed[AXIS_Z] * 100);
#endif
        gps_gspeed = (uint16_t)(sqrt(inter_mcu_state.speed[AXIS_X]*inter_mcu_state.speed[AXIS_X] + inter_mcu_state.speed[AXIS_Y]*inter_mcu_state.speed[AXIS_Y]) * 100);
        offset += XSENS_DATA_Velocity_LENGTH;
      }
      if (XSENS_MASK_Status(xsens_output_mode)) {
        xsens_msg_status = XSENS_DATA_Status_status(xsens_msg_buf,offset);
        if (bit_is_set(xsens_msg_status,2)) gps_mode = 0x03; // gps fix
        else gps_mode = 0;
        offset += XSENS_DATA_Status_LENGTH;
      }
      if (XSENS_MASK_TimeStamp(xsens_output_settings)) {
        xsens_time_stamp = XSENS_DATA_TimeStamp_ts(xsens_msg_buf,offset);
        gps_itow = xsens_time_stamp;
        offset += XSENS_DATA_TimeStamp_LENGTH;
      }
    //}
  }
  //DOWNLINK_SEND_QUAD_INS(&offset,&xsens_len,&xsens_msg_status,&xsens_time_stamp);
}


void parse_ins_buffer( uint8_t c ) {
  ck += c;
  switch (xsens_status) {
  case UNINIT:
    if (c != XSENS_START)
      goto error;
    xsens_status++;
    ck = 0;
    break;
  case GOT_START:
    if (c != XSENS_BID)
      goto error;
    xsens_status++;
    break;
  case GOT_BID:
    xsens_id = c;
    xsens_status++;
    break;
  case GOT_MID:
    xsens_len = c;
    if (xsens_len > XSENS_MAX_PAYLOAD)
      goto error;
    xsens_msg_idx = 0;
    xsens_status++;
    break;
  case GOT_LEN:
    xsens_msg_buf[xsens_msg_idx] = c;
    xsens_msg_idx++;
    if (xsens_msg_idx >= xsens_len)
      xsens_status++;
    break;
  case GOT_DATA:
    if (ck != 0)
      goto error;
    ins_msg_received = TRUE;
    goto restart;
    break;
  }
  return;
 error:  
 restart:
  xsens_status = UNINIT;
  return;
}
