/*
 * Copyright (C) 2013  Christophe De Wagter
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
 * @file modules/ins/xsens700.c
 * Parser for the Xsens700 protocol.
 */

#include "xsens700.h"

#include "led.h"
#include "generated/airframe.h"
#include "mcu_periph/uart.h"

#if USE_GPS_XSENS
#include "math/pprz_geodetic_wgs84.h"
#endif


uint8_t xsens_errorcode;
uint32_t xsens_msg_statusword;
uint16_t xsens_time_stamp;
uint16_t xsens_output_mode;
uint32_t xsens_output_settings;

float xsens_gps_arm_x = 0;
float xsens_gps_arm_y = 0;
float xsens_gps_arm_z = 0;

struct Xsens xsens700;

volatile int xsens_configured = 0;

void xsens700_init(void)
{
  xsens700.parser.status = UNINIT;
  xsens_configured = 30;

  xsens_msg_statusword = 0;
  xsens_time_stamp = 0;
}

static void xsens_ask_message_rate(uint8_t c1, uint8_t c2, uint8_t freq)
{
  uint8_t foo = 0;
  XsensSend1ByAddr(&c1);
  XsensSend1ByAddr(&c2);
  XsensSend1ByAddr(&foo);
  XsensSend1ByAddr(&freq);
}

void xsens700_periodic(void)
{
  if (xsens_configured > 0) {
    switch (xsens_configured) {
      case 25:
        /* send mode and settings to MT */
        XSENS_GoToConfig();
        //XSENS_SetOutputMode(xsens_output_mode);
        //XSENS_SetOutputSettings(xsens_output_settings);
        break;
      case 18:
        // Give pulses on SyncOut
        //XSENS_SetSyncOutSettings(0,0x0002);
        break;
      case 17:

        XsensHeader(XSENS_SetOutputConfiguration_ID, 44);
        xsens_ask_message_rate(0x10, 0x10, 4);    // UTC
        xsens_ask_message_rate(0x20, 0x30, 100);  // Attitude Euler
        xsens_ask_message_rate(0x40, 0x10, 100);  // Delta-V
        xsens_ask_message_rate(0x80, 0x20, 100);  // Rate of turn
        xsens_ask_message_rate(0xE0, 0x20, 50);   // Status
        xsens_ask_message_rate(0x30, 0x10, 50);   // Baro Pressure
        xsens_ask_message_rate(0x88, 0x40, 1);    // NavSol
        xsens_ask_message_rate(0x88, 0xA0, 1);    // SV Info
        xsens_ask_message_rate(0x50, 0x20, 50);   // GPS Altitude Ellipsiod
        xsens_ask_message_rate(0x50, 0x40, 50);   // GPS Position
        xsens_ask_message_rate(0xD0, 0x10, 50);   // GPS Speed
        XsensTrailer();
        break;
      case 2:
        //XSENS_ReqLeverArmGps();
        break;

      case 6:
        //XSENS_ReqMagneticDeclination();
        break;

      case 13:
        //#ifdef AHRS_H_X
        //#pragma message "Sending XSens Magnetic Declination."
        //xsens_declination = atan2(AHRS_H_Y, AHRS_H_X);
        //XSENS_SetMagneticDeclination(xsens_declination);
        //#endif
        break;
      case 12:
#ifdef GPS_IMU_LEVER_ARM_X
#pragma message "Sending XSens GPS Arm."
        XSENS_SetLeverArmGps(GPS_IMU_LEVER_ARM_X, GPS_IMU_LEVER_ARM_Y, GPS_IMU_LEVER_ARM_Z);
#endif
        break;
      case 10: {
        uint8_t baud = 1;
        XSENS_SetBaudrate(baud);
      }
      break;

      case 1:
        XSENS_GoToMeasurment();
        break;
      default:
        break;
    }
    xsens_configured--;
    return;
  }

}


void parse_xsens700_msg(void)
{
  uint8_t offset = 0;
  if (xsens700.parser.id == XSENS_ReqLeverArmGpsAck_ID) {
    xsens_gps_arm_x = XSENS_ReqLeverArmGpsAck_x(xsens700.parser.msg_buf);
    xsens_gps_arm_y = XSENS_ReqLeverArmGpsAck_y(xsens700.parser.msg_buf);
    xsens_gps_arm_z = XSENS_ReqLeverArmGpsAck_z(xsens700.parser.msg_buf);
  } else if (xsens700.parser.id == XSENS_Error_ID) {
    xsens_errorcode = XSENS_Error_errorcode(xsens700.parser.msg_buf);
  } else if (xsens700.parser.id == XSENS_MTData2_ID) {
    for (offset = 0; offset < xsens700.parser.len;) {
      uint8_t code1 = xsens700.parser.msg_buf[offset];
      uint8_t code2 = xsens700.parser.msg_buf[offset + 1];
      int subpacklen = (int)xsens700.parser.msg_buf[offset + 2];
      offset += 3;

      if (code1 == 0x10) {
        if (code2 == 0x10) {
          // UTC
        } else if (code2 == 0x20) {
          // Packet Counter
        }
        if (code2 == 0x30) {
          // ITOW
        }
      } else if (code1 == 0x20) {
        if (code2 == 0x30) {
          // Attitude Euler
          xsens700.euler.phi   = XSENS_DATA_Euler_roll(xsens700.parser.msg_buf, offset) * M_PI / 180;
          xsens700.euler.theta = XSENS_DATA_Euler_pitch(xsens700.parser.msg_buf, offset) * M_PI / 180;
          xsens700.euler.psi   = XSENS_DATA_Euler_yaw(xsens700.parser.msg_buf, offset) * M_PI / 180;

          xsens700.new_attitude = TRUE;
        }
      } else if (code1 == 0x40) {
        if (code2 == 0x10) {
          // Delta-V
          xsens700.accel.x = XSENS_DATA_Acceleration_x(xsens700.parser.msg_buf, offset) * 100.0f;
          xsens700.accel.y = XSENS_DATA_Acceleration_y(xsens700.parser.msg_buf, offset) * 100.0f;
          xsens700.accel.z = XSENS_DATA_Acceleration_z(xsens700.parser.msg_buf, offset) * 100.0f;
        }
      } else if (code1 == 0x80) {
        if (code2 == 0x20) {
          // Rate Of Turn
          xsens700.gyro.p = XSENS_DATA_RateOfTurn_x(xsens700.parser.msg_buf, offset) * M_PI / 180;
          xsens700.gyro.q = XSENS_DATA_RateOfTurn_y(xsens700.parser.msg_buf, offset) * M_PI / 180;
          xsens700.gyro.r = XSENS_DATA_RateOfTurn_z(xsens700.parser.msg_buf, offset) * M_PI / 180;
        }
      } else if (code1 == 0x30) {
        if (code2 == 0x10) {
          // Baro Pressure
        }
      } else if (code1 == 0xE0) {
        if (code2 == 0x20) {
          // Status Word
          xsens_msg_statusword = XSENS_DATA_StatusWord_status(xsens700.parser.msg_buf, offset);
          //xsens700.gps.tow = xsens_msg_statusword;
#if USE_GPS_XSENS
          if (bit_is_set(xsens_msg_statusword, 2) && bit_is_set(xsens_msg_statusword, 1)) {
            if (bit_is_set(xsens_msg_statusword, 23) && bit_is_set(xsens_msg_statusword, 24)) {
              xsens700.gps.fix = GPS_FIX_3D;
            } else {
              xsens700.gps.fix = GPS_FIX_2D;
            }
          } else { xsens700.gps.fix = GPS_FIX_NONE; }
#endif
        }
      } else if (code1 == 0x88) {
        if (code2 == 0x40) {
          xsens700.gps.week = XSENS_DATA_GpsSol_week(xsens700.parser.msg_buf, offset);
          xsens700.gps.num_sv = XSENS_DATA_GpsSol_numSv(xsens700.parser.msg_buf, offset);
          xsens700.gps.pacc = XSENS_DATA_GpsSol_pAcc(xsens700.parser.msg_buf, offset);
          xsens700.gps.sacc = XSENS_DATA_GpsSol_sAcc(xsens700.parser.msg_buf, offset);
          xsens700.gps.pdop = XSENS_DATA_GpsSol_pDop(xsens700.parser.msg_buf, offset);
        } else if (code2 == 0xA0) {
          // SVINFO
          xsens700.gps.tow = XSENS_XDI_GpsSvInfo_iTOW(xsens700.parser.msg_buf + offset);

#if USE_GPS_XSENS
          xsens700.gps.nb_channels = XSENS_XDI_GpsSvInfo_nch(xsens700.parser.msg_buf + offset);

          xsens700.gps.last_3dfix_ticks = sys_time.nb_sec_rem;
          xsens700.gps.last_3dfix_time = sys_time.nb_sec;

          uint8_t i;
          // Do not write outside buffer
          for (i = 0; i < Min(xsens700.gps.nb_channels, GPS_NB_CHANNELS); i++) {
            uint8_t ch = XSENS_XDI_GpsSvInfo_chn(xsens700.parser.msg_buf + offset, i);
            if (ch > xsens700.gps.nb_channels) { continue; }
            xsens700.gps.svinfos[ch].svid = XSENS_XDI_GpsSvInfo_svid(xsens700.parser.msg_buf + offset, i);
            xsens700.gps.svinfos[ch].flags = XSENS_XDI_GpsSvInfo_bitmask(xsens700.parser.msg_buf + offset, i);
            xsens700.gps.svinfos[ch].qi = XSENS_XDI_GpsSvInfo_qi(xsens700.parser.msg_buf + offset, i);
            xsens700.gps.svinfos[ch].cno = XSENS_XDI_GpsSvInfo_cnr(xsens700.parser.msg_buf + offset, i);
          }
#endif
        }
      } else if (code1 == 0x50) {
        if (code2 == 0x10) {
          //xsens700.gps.hmsl = XSENS_DATA_Altitude_h(xsens700.parser.msg_buf,offset)* 1000.0f;
        } else if (code2 == 0x20) {
          // Altitude Elipsoid
          xsens700.gps.lla_pos.alt = XSENS_DATA_Altitude_h(xsens700.parser.msg_buf, offset) * 1000.0f;

          // Compute geoid (MSL) height
          float geoid_h = wgs84_ellipsoid_to_geoid_f(xsens700.lla_f.lat, xsens700.lla_f.lon);
          xsens700.gps.hmsl =  xsens700.gps.lla_pos.alt - (geoid_h * 1000.0f);
          SetBit(xsens700.gps.valid_fields, GPS_VALID_HMSL_BIT);

          //xsens700.gps.tow = geoid_h * 1000.0f; //xsens700.gps.utm_pos.alt;
        } else if (code2 == 0x40) {
          // LatLong
#ifdef GPS_LED
          LED_TOGGLE(GPS_LED);
#endif
          xsens700.gps.last_3dfix_ticks = sys_time.nb_sec_rem;
          xsens700.gps.last_3dfix_time = sys_time.nb_sec;
          xsens700.gps.week = 0; // FIXME

          xsens700.lla_f.lat = RadOfDeg(XSENS_DATA_LatLon_lat(xsens700.parser.msg_buf, offset));
          xsens700.lla_f.lon = RadOfDeg(XSENS_DATA_LatLon_lon(xsens700.parser.msg_buf, offset));
        }
      } else if (code1 == 0xD0) {
        if (code2 == 0x10) {
          // Velocity
          xsens700.vel.x = XSENS_DATA_VelocityXYZ_x(xsens700.parser.msg_buf, offset);
          xsens700.vel.y = XSENS_DATA_VelocityXYZ_y(xsens700.parser.msg_buf, offset);
          xsens700.vel.z = XSENS_DATA_VelocityXYZ_z(xsens700.parser.msg_buf, offset);
          xsens700.gps.ned_vel.x = xsens700.vel.x;
          xsens700.gps.ned_vel.y = xsens700.vel.y;
          xsens700.gps.ned_vel.z = xsens700.vel.x;
          SetBit(xsens700.gps.valid_fields, GPS_VALID_VEL_NED_BIT);
        }
      }

      if (subpacklen < 0) {
        subpacklen = 0;
      }
      offset += subpacklen;
    }
  }
}
