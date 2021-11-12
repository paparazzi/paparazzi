/*
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
 */

/** @file modules/ins/xsens.c
 * Parser for the Xsens protocol.
 */

#include "xsens.h"

#include "generated/airframe.h"
#include "led.h"

#if USE_GPS_XSENS
#include "math/pprz_geodetic_wgs84.h"
#endif

// FIXME Debugging Only
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"


/* output mode : calibrated, orientation, position, velocity, status
 * -----------
 *
 * bit 0 temp
 * bit 1 calibrated
 * bit 2 orentation
 * bit 3 aux
 *
 * bit 4 position
 * bit 5 velocity
 * bit 6-7 Reserved
 *
 * bit 8-10 Reserved
 * bit 11 status
 *
 * bit 12 GPS PVT+baro
 * bit 13 Reserved
 * bit 14 Raw
 * bit 15 Reserved
 */

#ifndef XSENS_OUTPUT_MODE
#define XSENS_OUTPUT_MODE 0x1836
#endif
/* output settings : timestamp, euler, acc, rate, mag, float, no aux, lla, m/s, NED
 * -----------
 *
 * bit 01 0=none, 1=sample counter, 2=utc, 3=sample+utc
 * bit 23 0=quaternion, 1=euler, 2=DCM
 *
 * bit 4 1=disable acc output
 * bit 5 1=disable gyro output
 * bit 6 1=disable magneto output
 * bit 7 Reserved
 *
 * bit 89 0=float, 1=fixedpoint12.20, 2=fixedpoint16.32
 * bit 10 1=disable aux analog 1
 * bit 11 1=disable aux analog 2
 *
 * bit 12-15 0-only: 14-16 WGS84
 *
 * bit 16-19 0-only: 16-18 m/s XYZ
 *
 * bit 20-23 Reserved
 *
 * bit 24-27 Reseverd
 *
 * bit 28-30 Reseverd
 * bit 31 0=X-North-Z-Up, 1=North-East-Down
 */
#ifndef XSENS_OUTPUT_SETTINGS
#define XSENS_OUTPUT_SETTINGS 0x80000C05
#endif

uint8_t xsens_errorcode;
uint8_t xsens_msg_status;
uint16_t xsens_time_stamp;
uint16_t xsens_output_mode;
uint32_t xsens_output_settings;


float xsens_declination = 0;
float xsens_gps_arm_x = 0;
float xsens_gps_arm_y = 0;
float xsens_gps_arm_z = 0;

volatile int xsens_configured = 0;

struct Xsens xsens;

void parse_xsens_buffer(uint8_t c);

void xsens_init(void)
{
  xsens.parser.status = UNINIT;
  xsens_configured = 20;

  xsens_msg_status = 0;
  xsens_time_stamp = 0;
  xsens_output_mode = XSENS_OUTPUT_MODE;
  xsens_output_settings = XSENS_OUTPUT_SETTINGS;
}

void xsens_periodic(void)
{
  if (xsens_configured > 0) {
    switch (xsens_configured) {
      case 20:
        /* send mode and settings to MT */
        XSENS_GoToConfig();
        XSENS_SetOutputMode(xsens_output_mode);
        XSENS_SetOutputSettings(xsens_output_settings);
        break;
      case 18:
        // Give pulses on SyncOut
        XSENS_SetSyncOutSettings(0, 0x0002);
        break;
      case 17:
        // 1 pulse every 100 samples
        XSENS_SetSyncOutSettings(1, 100);
        break;
      case 2:
        XSENS_ReqLeverArmGps();
        break;

      case 6:
        XSENS_ReqMagneticDeclination();
        break;

      case 13:
#ifdef AHRS_H_X
#pragma message "Sending XSens Magnetic Declination."
        xsens_declination = atan2(AHRS_H_Y, AHRS_H_X);
        XSENS_SetMagneticDeclination(xsens_declination);
#endif
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

  RunOnceEvery(100, XSENS_ReqGPSStatus());
}


void parse_xsens_msg(void)
{
  uint8_t offset = 0;
  if (xsens.parser.id == XSENS_ReqOutputModeAck_ID) {
    xsens_output_mode = XSENS_ReqOutputModeAck_mode(xsens.parser.msg_buf);
  } else if (xsens.parser.id == XSENS_ReqOutputSettings_ID) {
    xsens_output_settings = XSENS_ReqOutputSettingsAck_settings(xsens.parser.msg_buf);
  } else if (xsens.parser.id == XSENS_ReqMagneticDeclinationAck_ID) {
    xsens_declination = DegOfRad(XSENS_ReqMagneticDeclinationAck_declination(xsens.parser.msg_buf));

    DOWNLINK_SEND_IMU_MAG_SETTINGS(DefaultChannel, DefaultDevice, &xsens_declination, &xsens_declination, &xsens_gps_arm_x,
                                   &xsens_gps_arm_y, &xsens_gps_arm_z);
  } else if (xsens.parser.id == XSENS_ReqLeverArmGpsAck_ID) {
    xsens_gps_arm_x = XSENS_ReqLeverArmGpsAck_x(xsens.parser.msg_buf);
    xsens_gps_arm_y = XSENS_ReqLeverArmGpsAck_y(xsens.parser.msg_buf);
    xsens_gps_arm_z = XSENS_ReqLeverArmGpsAck_z(xsens.parser.msg_buf);

    DOWNLINK_SEND_IMU_MAG_SETTINGS(DefaultChannel, DefaultDevice, &xsens_declination, &xsens_declination, &xsens_gps_arm_x,
                                   &xsens_gps_arm_y, &xsens_gps_arm_z);
  } else if (xsens.parser.id == XSENS_Error_ID) {
    xsens_errorcode = XSENS_Error_errorcode(xsens.parser.msg_buf);
  }
#if USE_GPS_XSENS
  else if (xsens.parser.id == XSENS_GPSStatus_ID) {
    xsens.gps.nb_channels = XSENS_GPSStatus_nch(xsens.parser.msg_buf);
    xsens.gps.num_sv = 0;

    uint8_t i;
    // Do not write outside buffer
    for (i = 0; i < Min(xsens.gps.nb_channels, GPS_NB_CHANNELS); i++) {
      uint8_t ch = XSENS_GPSStatus_chn(xsens.parser.msg_buf, i);
      if (ch > xsens.gps.nb_channels) { continue; }
      xsens.gps.svinfos[ch].svid = XSENS_GPSStatus_svid(xsens.parser.msg_buf, i);
      xsens.gps.svinfos[ch].flags = XSENS_GPSStatus_bitmask(xsens.parser.msg_buf, i);
      xsens.gps.svinfos[ch].qi = XSENS_GPSStatus_qi(xsens.parser.msg_buf, i);
      xsens.gps.svinfos[ch].cno = XSENS_GPSStatus_cnr(xsens.parser.msg_buf, i);
      if (xsens.gps.svinfos[ch].flags > 0) {
        xsens.gps.num_sv++;
      }
    }
  }
#endif
  else if (xsens.parser.id == XSENS_MTData_ID) {
    /* test RAW modes else calibrated modes */
    //if ((XSENS_MASK_RAWInertial(xsens_output_mode)) || (XSENS_MASK_RAWGPS(xsens_output_mode))) {
    if (XSENS_MASK_RAWInertial(xsens_output_mode)) {
      /* should we write raw data to separate struct? */
      xsens.gyro.p = XSENS_DATA_RAWInertial_gyrX(xsens.parser.msg_buf, offset);
      xsens.gyro.q = XSENS_DATA_RAWInertial_gyrY(xsens.parser.msg_buf, offset);
      xsens.gyro.r = XSENS_DATA_RAWInertial_gyrZ(xsens.parser.msg_buf, offset);
      xsens.gyro_available = TRUE;
      offset += XSENS_DATA_RAWInertial_LENGTH;
    }
    if (XSENS_MASK_RAWGPS(xsens_output_mode)) {
#if USE_GPS_XSENS_RAW_DATA && USE_GPS_XSENS
      xsens.gps.week = 0; // FIXME
      xsens.gps.tow = XSENS_DATA_RAWGPS_itow(xsens.parser.msg_buf, offset) * 10;
      xsens.gps.lla_pos.lat = XSENS_DATA_RAWGPS_lat(xsens.parser.msg_buf, offset);
      xsens.gps.lla_pos.lon = XSENS_DATA_RAWGPS_lon(xsens.parser.msg_buf, offset);
      xsens.gps.lla_pos.alt = XSENS_DATA_RAWGPS_alt(xsens.parser.msg_buf, offset);
      SetBit(xsens.gps.valid_fields, GPS_VALID_POS_LLA_BIT);

      // Compute geoid (MSL) height
      uint32_t hmsl = wgs84_ellipsoid_to_geoid_i(xsens.gps.lla_pos.lat, xsens.gps.lla_pos.lon);
      xsens.gps.hmsl =  XSENS_DATA_RAWGPS_alt(xsens.parser.msg_buf, offset) - hmsl;
      SetBit(gps.valid_fields, GPS_VALID_HMSL_BIT);

      xsens.gps.ned_vel.x = XSENS_DATA_RAWGPS_vel_n(xsens.parser.msg_buf, offset);
      xsens.gps.ned_vel.y = XSENS_DATA_RAWGPS_vel_e(xsens.parser.msg_buf, offset);
      xsens.gps.ned_vel.z = XSENS_DATA_RAWGPS_vel_d(xsens.parser.msg_buf, offset);
      SetBit(gps.valid_fields, GPS_VALID_VEL_NED_BIT);
      xsens.gps.pacc = XSENS_DATA_RAWGPS_hacc(xsens.parser.msg_buf, offset) / 100;
      xsens.gps.sacc = XSENS_DATA_RAWGPS_sacc(xsens.parser.msg_buf, offset) / 100;
      xsens.gps.pdop = 5;  // FIXME Not output by XSens

      xsens.gps_available = TRUE;
#endif
      offset += XSENS_DATA_RAWGPS_LENGTH;
    }
    //} else {
    if (XSENS_MASK_Temp(xsens_output_mode)) {
      offset += XSENS_DATA_Temp_LENGTH;
    }
    if (XSENS_MASK_Calibrated(xsens_output_mode)) {
      uint8_t l = 0;
      if (!XSENS_MASK_AccOut(xsens_output_settings)) {
        xsens.accel.x = XSENS_DATA_Calibrated_accX(xsens.parser.msg_buf, offset);
        xsens.accel.y = XSENS_DATA_Calibrated_accY(xsens.parser.msg_buf, offset);
        xsens.accel.z = XSENS_DATA_Calibrated_accZ(xsens.parser.msg_buf, offset);
        xsens.accel_available = TRUE;
        l++;
      }
      if (!XSENS_MASK_GyrOut(xsens_output_settings)) {
        xsens.gyro.p = XSENS_DATA_Calibrated_gyrX(xsens.parser.msg_buf, offset);
        xsens.gyro.q = XSENS_DATA_Calibrated_gyrY(xsens.parser.msg_buf, offset);
        xsens.gyro.r = XSENS_DATA_Calibrated_gyrZ(xsens.parser.msg_buf, offset);
        xsens.gyro_available = TRUE;
        l++;
      }
      if (!XSENS_MASK_MagOut(xsens_output_settings)) {
        xsens.mag.x = XSENS_DATA_Calibrated_magX(xsens.parser.msg_buf, offset);
        xsens.mag.y = XSENS_DATA_Calibrated_magY(xsens.parser.msg_buf, offset);
        xsens.mag.z = XSENS_DATA_Calibrated_magZ(xsens.parser.msg_buf, offset);
        xsens.mag_available = TRUE;
        l++;
      }
      offset += l * XSENS_DATA_Calibrated_LENGTH / 3;
    }
    if (XSENS_MASK_Orientation(xsens_output_mode)) {
      if (XSENS_MASK_OrientationMode(xsens_output_settings) == 0x00) {
        xsens.quat.qi = XSENS_DATA_Quaternion_q0(xsens.parser.msg_buf, offset);
        xsens.quat.qx = XSENS_DATA_Quaternion_q1(xsens.parser.msg_buf, offset);
        xsens.quat.qy = XSENS_DATA_Quaternion_q2(xsens.parser.msg_buf, offset);
        xsens.quat.qz = XSENS_DATA_Quaternion_q3(xsens.parser.msg_buf, offset);
        //float_eulers_of_quat(&xsens.euler, &xsens.quat);
        offset += XSENS_DATA_Quaternion_LENGTH;
      }
      if (XSENS_MASK_OrientationMode(xsens_output_settings) == 0x01) {
        xsens.euler.phi   = XSENS_DATA_Euler_roll(xsens.parser.msg_buf, offset) * M_PI / 180;
        xsens.euler.theta = XSENS_DATA_Euler_pitch(xsens.parser.msg_buf, offset) * M_PI / 180;
        xsens.euler.psi   = XSENS_DATA_Euler_yaw(xsens.parser.msg_buf, offset) * M_PI / 180;
        offset += XSENS_DATA_Euler_LENGTH;
      }
      if (XSENS_MASK_OrientationMode(xsens_output_settings) == 0x10) {
        offset += XSENS_DATA_Matrix_LENGTH;
      }
      xsens.new_attitude = TRUE;
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
      xsens.lla_f.lat = RadOfDeg(XSENS_DATA_Position_lat(xsens.parser.msg_buf, offset));
      xsens.lla_f.lon = RadOfDeg(XSENS_DATA_Position_lon(xsens.parser.msg_buf, offset));
      xsens.lla_f.alt = XSENS_DATA_Position_alt(xsens.parser.msg_buf, offset);
      offset += XSENS_DATA_Position_LENGTH;

#if (! USE_GPS_XSENS_RAW_DATA) && USE_GPS_XSENS
      LLA_BFP_OF_REAL(xsens.gps.lla_pos, xsens.lla_f);
      SetBit(gps.valid_fields, GPS_VALID_POS_LLA_BIT);
      xsens.gps_available = TRUE;
#endif
    }
    if (XSENS_MASK_Velocity(xsens_output_mode)) {
      xsens.vel.x = XSENS_DATA_Velocity_vx(xsens.parser.msg_buf, offset);
      xsens.vel.y = XSENS_DATA_Velocity_vy(xsens.parser.msg_buf, offset);
      xsens.vel.z = XSENS_DATA_Velocity_vz(xsens.parser.msg_buf, offset);
      offset += XSENS_DATA_Velocity_LENGTH;
    }
    if (XSENS_MASK_Status(xsens_output_mode)) {
      xsens_msg_status = XSENS_DATA_Status_status(xsens.parser.msg_buf, offset);
#if USE_GPS_XSENS
      if (bit_is_set(xsens_msg_status, 2)) { xsens.gps.fix = GPS_FIX_3D; } // gps fix
      else if (bit_is_set(xsens_msg_status, 1)) { xsens.gps.fix = 0x01; } // efk valid
      else { xsens.gps.fix = GPS_FIX_NONE; }
#ifdef GPS_LED
      if (xsens.gps.fix == GPS_FIX_3D) {
        LED_ON(GPS_LED);
      } else {
        LED_TOGGLE(GPS_LED);
      }
#endif // GPS_LED
#endif //  USE_GPS_XSENS
      offset += XSENS_DATA_Status_LENGTH;
    }
    if (XSENS_MASK_TimeStamp(xsens_output_settings)) {
      xsens.time_stamp = XSENS_DATA_TimeStamp_ts(xsens.parser.msg_buf, offset);
      offset += XSENS_DATA_TimeStamp_LENGTH;
    }
    if (XSENS_MASK_UTC(xsens_output_settings)) {
      xsens.time.hour = XSENS_DATA_UTC_hour(xsens.parser.msg_buf, offset);
      xsens.time.min = XSENS_DATA_UTC_min(xsens.parser.msg_buf, offset);
      xsens.time.sec = XSENS_DATA_UTC_sec(xsens.parser.msg_buf, offset);
      xsens.time.nanosec = XSENS_DATA_UTC_nanosec(xsens.parser.msg_buf, offset);
      xsens.time.year = XSENS_DATA_UTC_year(xsens.parser.msg_buf, offset);
      xsens.time.month = XSENS_DATA_UTC_month(xsens.parser.msg_buf, offset);
      xsens.time.day = XSENS_DATA_UTC_day(xsens.parser.msg_buf, offset);

      offset += XSENS_DATA_UTC_LENGTH;
    }
  }

}
