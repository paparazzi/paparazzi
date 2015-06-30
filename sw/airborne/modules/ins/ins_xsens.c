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
 *
 */

/** @file ins_xsens.c
 * Parser for the Xsens protocol.
 */

#include "ins_module.h"
#include "ins_xsens.h"
#include "subsystems/ins.h"

#include <inttypes.h>

#include "generated/airframe.h"

#include "mcu_periph/sys_time.h"
#include "messages.h"

#if USE_GPS_XSENS
#if !USE_GPS
#error "USE_GPS needs to be 1 to use the Xsens GPS!"
#endif
#include "subsystems/gps.h"
#include "subsystems/abi.h"
#include "math/pprz_geodetic_wgs84.h"
#include "math/pprz_geodetic_float.h"
#include "subsystems/navigation/common_nav.h" /* needed for nav_utm_zone0 */
bool_t gps_xsens_msg_available;
#endif

// positions
INS_FORMAT ins_x;
INS_FORMAT ins_y;
INS_FORMAT ins_z;

// velocities
INS_FORMAT ins_vx;
INS_FORMAT ins_vy;
INS_FORMAT ins_vz;

// body angles
INS_FORMAT ins_phi;
INS_FORMAT ins_theta;
INS_FORMAT ins_psi;

// angle rates
INS_FORMAT ins_p;
INS_FORMAT ins_q;
INS_FORMAT ins_r;

// accelerations
INS_FORMAT ins_ax;
INS_FORMAT ins_ay;
INS_FORMAT ins_az;

// magnetic
INS_FORMAT ins_mx;
INS_FORMAT ins_my;
INS_FORMAT ins_mz;

#if USE_INS_MODULE
float ins_pitch_neutral;
float ins_roll_neutral;
#endif


//////////////////////////////////////////////////////////////////////////////////////////
//
//  XSens Specific
//

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

/* output mode : calibrated, orientation, position, velocity, status
 * -----------
 *
 *  bit 0 temp
 *  bit 1 calibrated
 *  bit 2 orentation
 *  bit 3 aux
 *
 *  bit 4 position
 *  bit 5 velocity
 *  bit 6-7 Reserved
 *
 *  bit 8-10 Reserved
 *  bit 11  status
 *
 *  bit 12  GPS PVT+baro
 *  bit 13  Reserved
 *  bit 14  Raw
 *  bit 15  Reserved
 */

#ifndef XSENS_OUTPUT_MODE
#define XSENS_OUTPUT_MODE 0x1836
#endif
/* output settings : timestamp, euler, acc, rate, mag, float, no aux, lla, m/s, NED
 * -----------
 *
 *  bit 01  0=none, 1=sample counter, 2=utc, 3=sample+utc
 *  bit 23  0=quaternion, 1=euler, 2=DCM
 *
 *  bit 4 1=disable acc output
 *  bit 5 1=disable gyro output
 *  bit 6 1=disable magneto output
 *  bit 7 Reserved
 *
 *  bit 89  0=float, 1=fixedpoint12.20, 2=fixedpoint16.32
 *  bit 10  1=disable aux analog 1
 *  bit 11  1=disable aux analog 2
 *
 *  bit 12-15 0-only: 14-16 WGS84
 *
 *  bit 16-19 0-only: 16-18 m/s XYZ
 *
 *  bit 20-23 Reserved
 *
 *  bit 24-27 Reseverd
 *
 *  bit 28-30 Reseverd
 *  bit 31  0=X-North-Z-Up, 1=North-East-Down
 */
#ifndef XSENS_OUTPUT_SETTINGS
#define XSENS_OUTPUT_SETTINGS 0x80000C05
#endif

#define UNINIT        0
#define GOT_START     1
#define GOT_BID       2
#define GOT_MID       3
#define GOT_LEN       4
#define GOT_DATA      5
#define GOT_CHECKSUM  6

// FIXME Debugging Only
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"


uint8_t xsens_errorcode;
uint8_t xsens_msg_status;
uint16_t xsens_time_stamp;
uint16_t xsens_output_mode;
uint32_t xsens_output_settings;


float xsens_declination = 0;
float xsens_gps_arm_x = 0;
float xsens_gps_arm_y = 0;
float xsens_gps_arm_z = 0;

#if USE_GPS_XSENS
struct LlaCoor_f lla_f;
struct UtmCoor_f utm_f;
#endif

struct XsensTime xsens_time;

static uint8_t xsens_id;
static uint8_t xsens_status;
static uint8_t xsens_len;
static uint8_t xsens_msg_idx;
static uint8_t ck;
uint8_t send_ck;

volatile int xsens_configured = 0;

void xsens_init(void);

void xsens_init(void)
{

  xsens_status = UNINIT;
  xsens_configured = 20;

#if USE_INS_MODULE
  ins_pitch_neutral = INS_PITCH_NEUTRAL_DEFAULT;
  ins_roll_neutral = INS_ROLL_NEUTRAL_DEFAULT;
#endif

  xsens_msg_status = 0;
  xsens_time_stamp = 0;
  xsens_output_mode = XSENS_OUTPUT_MODE;
  xsens_output_settings = XSENS_OUTPUT_SETTINGS;
}

#if USE_IMU
struct ImuXsens imu_xsens;

void imu_impl_init(void)
{
  xsens_init();
  imu_xsens.gyro_available = FALSE;
  imu_xsens.accel_available = FALSE;
  imu_xsens.mag_available = FALSE;
}

void imu_periodic(void)
{
  xsens_periodic();
}
#endif /* USE_IMU */

#if USE_INS_MODULE
void ins_xsens_update_gps(struct GpsState *gps_s);

void ins_xsens_init(void)
{
  struct UtmCoor_f utm0 = { nav_utm_north0, nav_utm_east0, 0., nav_utm_zone0 };
  stateSetLocalUtmOrigin_f(&utm0);
  stateSetPositionUtm_f(&utm0);

  xsens_init();
}

#include "subsystems/abi.h"
static abi_event gps_ev;
static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps_s)
{
  ins_xsens_update_gps(gps_s);
}

void ins_xsens_register(void)
{
  ins_register_impl(ins_xsens_init);
  AbiBindMsgGPS(ABI_BROADCAST, &gps_ev, gps_cb);
}

void ins_xsens_update_gps(struct GpsState *gps_s)
{
  struct UtmCoor_f utm;
  utm.east = gps_s->utm_pos.east / 100.;
  utm.north = gps_s->utm_pos.north / 100.;
  utm.zone = nav_utm_zone0;
  utm.alt = gps_s->hmsl / 1000.;

  // set position
  stateSetPositionUtm_f(&utm);

  struct NedCoor_f ned_vel = {
    gps_s->ned_vel.x / 100.,
    gps_s->ned_vel.y / 100.,
    gps_s->ned_vel.z / 100.
  };
  // set velocity
  stateSetSpeedNed_f(&ned_vel);
}
#endif

#if USE_GPS_XSENS
void gps_impl_init(void)
{
  gps.nb_channels = 0;
}

static void gps_xsens_publish(void)
{
  // publish gps data
  uint32_t now_ts = get_sys_time_usec();
  gps.last_msg_ticks = sys_time.nb_sec_rem;
  gps.last_msg_time = sys_time.nb_sec;
  if (gps.fix == GPS_FIX_3D) {
    gps.last_3dfix_ticks = sys_time.nb_sec_rem;
    gps.last_3dfix_time = sys_time.nb_sec;
  }
  AbiSendMsgGPS(GPS_XSENS_ID, now_ts, &gps);
}
#endif

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

#if USE_INS_MODULE
#include "state.h"

static inline void update_state_interface(void)
{
  // Send to Estimator (Control)
#ifdef XSENS_BACKWARDS
  struct FloatEulers att = {
    -ins_phi + ins_roll_neutral,
    -ins_theta + ins_pitch_neutral,
    ins_psi + RadOfDeg(180)
  };
  struct FloatRates rates = {
    -ins_p,
    -ins_q,
    ins_r
  };
#else
  struct FloatEulers att = {
    ins_phi + ins_roll_neutral,
    ins_theta + ins_pitch_neutral,
    ins_psi
  };
  struct FloatRates rates = {
    ins_p,
    ins_q,
    ins_r
  };
#endif
  stateSetNedToBodyEulers_f(&att);
  stateSetBodyRates_f(&rates);
}
#endif /* USE_INS_MODULE */

void handle_ins_msg(void)
{

#if USE_INS_MODULE
  update_state_interface();
#endif

#if USE_IMU
  uint32_t now_ts = get_sys_time_usec();
#ifdef XSENS_BACKWARDS
  if (imu_xsens.gyro_available) {
    RATES_ASSIGN(imu.gyro_unscaled, -RATE_BFP_OF_REAL(ins_p), -RATE_BFP_OF_REAL(ins_q), RATE_BFP_OF_REAL(ins_r));
     imu_xsens.gyro_available = FALSE;
     imu_scale_gyro(&imu);
     AbiSendMsgIMU_GYRO_INT32(IMU_XSENS_ID, now_ts, &imu.gyro);
  }
  if (imu_xsens.accel_available) {
    VECT3_ASSIGN(imu.accel_unscaled, -ACCEL_BFP_OF_REAL(ins_ax), -ACCEL_BFP_OF_REAL(ins_ay), ACCEL_BFP_OF_REAL(ins_az));
    imu_xsens.accel_available = FALSE;
    imu_scale_accel(&imu);
    AbiSendMsgIMU_ACCEL_INT32(IMU_XSENS_ID, now_ts, &imu.accel);
  }
  if (imu_xsens.mag_available) {
    VECT3_ASSIGN(imu.mag_unscaled, -MAG_BFP_OF_REAL(ins_mx), -MAG_BFP_OF_REAL(ins_my), MAG_BFP_OF_REAL(ins_mz));
    imu_xsens.mag_available = FALSE;
    imu_scale_mag(&imu);
    AbiSendMsgIMU_MAG_INT32(IMU_XSENS_ID, now_ts, &imu.mag);
  }
#else
  if (imu_xsens.gyro_available) {
    RATES_ASSIGN(imu.gyro_unscaled, RATE_BFP_OF_REAL(ins_p), RATE_BFP_OF_REAL(ins_q), RATE_BFP_OF_REAL(ins_r));
    imu_xsens.gyro_available = FALSE;
    imu_scale_gyro(&imu);
    AbiSendMsgIMU_GYRO_INT32(IMU_XSENS_ID, now_ts, &imu.gyro);
  }
  if (imu_xsens.accel_available) {
    VECT3_ASSIGN(imu.accel_unscaled, ACCEL_BFP_OF_REAL(ins_ax), ACCEL_BFP_OF_REAL(ins_ay), ACCEL_BFP_OF_REAL(ins_az));
    imu_xsens.accel_available = FALSE;
    imu_scale_accel(&imu);
    AbiSendMsgIMU_ACCEL_INT32(IMU_XSENS_ID, now_ts, &imu.accel);
  }
  if (imu_xsens.mag_available) {
    VECT3_ASSIGN(imu.mag_unscaled, MAG_BFP_OF_REAL(ins_mx), MAG_BFP_OF_REAL(ins_my), MAG_BFP_OF_REAL(ins_mz));
    imu_xsens.mag_available = FALSE;
    imu_scale_mag(&imu);
    AbiSendMsgIMU_MAG_INT32(IMU_XSENS_ID, now_ts, &imu.mag);
  }
#endif /* XSENS_BACKWARDS */
#endif /* USE_IMU */

#if USE_GPS_XSENS

  // Horizontal speed
  float fspeed = sqrt(ins_vx * ins_vx + ins_vy * ins_vy);
  if (gps.fix != GPS_FIX_3D) {
    fspeed = 0;
  }
  gps.gspeed = fspeed * 100.;
  gps.speed_3d = (uint16_t)(sqrt(ins_vx * ins_vx + ins_vy * ins_vy + ins_vz * ins_vz) * 100);

  float fcourse = atan2f((float)ins_vy, (float)ins_vx);
  gps.course = fcourse * 1e7;
#endif // USE_GPS_XSENS
}

void parse_ins_msg(void)
{
  uint8_t offset = 0;
  if (xsens_id == XSENS_ReqOutputModeAck_ID) {
    xsens_output_mode = XSENS_ReqOutputModeAck_mode(xsens_msg_buf);
  } else if (xsens_id == XSENS_ReqOutputSettings_ID) {
    xsens_output_settings = XSENS_ReqOutputSettingsAck_settings(xsens_msg_buf);
  } else if (xsens_id == XSENS_ReqMagneticDeclinationAck_ID) {
    xsens_declination = DegOfRad(XSENS_ReqMagneticDeclinationAck_declination(xsens_msg_buf));

    DOWNLINK_SEND_IMU_MAG_SETTINGS(DefaultChannel, DefaultDevice, &xsens_declination, &xsens_declination, &xsens_gps_arm_x,
                                   &xsens_gps_arm_y, &xsens_gps_arm_z);
  } else if (xsens_id == XSENS_ReqLeverArmGpsAck_ID) {
    xsens_gps_arm_x = XSENS_ReqLeverArmGpsAck_x(xsens_msg_buf);
    xsens_gps_arm_y = XSENS_ReqLeverArmGpsAck_y(xsens_msg_buf);
    xsens_gps_arm_z = XSENS_ReqLeverArmGpsAck_z(xsens_msg_buf);

    DOWNLINK_SEND_IMU_MAG_SETTINGS(DefaultChannel, DefaultDevice, &xsens_declination, &xsens_declination, &xsens_gps_arm_x,
                                   &xsens_gps_arm_y, &xsens_gps_arm_z);
  } else if (xsens_id == XSENS_Error_ID) {
    xsens_errorcode = XSENS_Error_errorcode(xsens_msg_buf);
  }
#if USE_GPS_XSENS
  else if (xsens_id == XSENS_GPSStatus_ID) {
    gps.nb_channels = XSENS_GPSStatus_nch(xsens_msg_buf);
    gps.num_sv = 0;

    uint8_t i;
    // Do not write outside buffer
    for (i = 0; i < Min(gps.nb_channels, GPS_NB_CHANNELS); i++) {
      uint8_t ch = XSENS_GPSStatus_chn(xsens_msg_buf, i);
      if (ch > gps.nb_channels) { continue; }
      gps.svinfos[ch].svid = XSENS_GPSStatus_svid(xsens_msg_buf, i);
      gps.svinfos[ch].flags = XSENS_GPSStatus_bitmask(xsens_msg_buf, i);
      gps.svinfos[ch].qi = XSENS_GPSStatus_qi(xsens_msg_buf, i);
      gps.svinfos[ch].cno = XSENS_GPSStatus_cnr(xsens_msg_buf, i);
      if (gps.svinfos[ch].flags > 0) {
        gps.num_sv++;
      }
    }
  }
#endif
  else if (xsens_id == XSENS_MTData_ID) {
    /* test RAW modes else calibrated modes */
    //if ((XSENS_MASK_RAWInertial(xsens_output_mode)) || (XSENS_MASK_RAWGPS(xsens_output_mode))) {
    if (XSENS_MASK_RAWInertial(xsens_output_mode)) {
      ins_p = XSENS_DATA_RAWInertial_gyrX(xsens_msg_buf, offset);
      ins_q = XSENS_DATA_RAWInertial_gyrY(xsens_msg_buf, offset);
      ins_r = XSENS_DATA_RAWInertial_gyrZ(xsens_msg_buf, offset);
#if USE_IMU
      imu_xsens.gyro_available = TRUE;
#endif
      offset += XSENS_DATA_RAWInertial_LENGTH;
    }
    if (XSENS_MASK_RAWGPS(xsens_output_mode)) {
#if USE_GPS_XSENS_RAW_DATA && USE_GPS_XSENS

      gps.week = 0; // FIXME
      gps.tow = XSENS_DATA_RAWGPS_itow(xsens_msg_buf, offset) * 10;
      gps.lla_pos.lat = XSENS_DATA_RAWGPS_lat(xsens_msg_buf, offset);
      gps.lla_pos.lon = XSENS_DATA_RAWGPS_lon(xsens_msg_buf, offset);
      gps.lla_pos.alt = XSENS_DATA_RAWGPS_alt(xsens_msg_buf, offset);

      /* Set the real UTM zone */
      gps.utm_pos.zone = (gps.lla_pos.lon / 1e7 + 180) / 6 + 1;
      LLA_FLOAT_OF_BFP(lla_f, gps.lla_pos);
      utm_f.zone = nav_utm_zone0;
      /* convert to utm */
      utm_of_lla_f(&utm_f, &lla_f);
      /* copy results of utm conversion */
      gps.utm_pos.east = utm_f.east * 100;
      gps.utm_pos.north = utm_f.north * 100;
      gps.utm_pos.alt = gps.lla_pos.alt;

      ins_x = utm_f.east;
      ins_y = utm_f.north;
      // Altitude: Xsens LLH gives ellipsoid height
      ins_z = -(INS_FORMAT)XSENS_DATA_RAWGPS_alt(xsens_msg_buf, offset) / 1000.;

      // Compute geoid (MSL) height
      float hmsl = wgs84_ellipsoid_to_geoid(lla_f.lat, lla_f.lon);
      gps.hmsl =  XSENS_DATA_RAWGPS_alt(xsens_msg_buf, offset) - (hmsl * 1000.0f);

      ins_vx = ((INS_FORMAT)XSENS_DATA_RAWGPS_vel_n(xsens_msg_buf, offset)) / 100.;
      ins_vy = ((INS_FORMAT)XSENS_DATA_RAWGPS_vel_e(xsens_msg_buf, offset)) / 100.;
      ins_vz = ((INS_FORMAT)XSENS_DATA_RAWGPS_vel_d(xsens_msg_buf, offset)) / 100.;
      gps.ned_vel.x = XSENS_DATA_RAWGPS_vel_n(xsens_msg_buf, offset);
      gps.ned_vel.y = XSENS_DATA_RAWGPS_vel_e(xsens_msg_buf, offset);
      gps.ned_vel.z = XSENS_DATA_RAWGPS_vel_d(xsens_msg_buf, offset);
      gps.pacc = XSENS_DATA_RAWGPS_hacc(xsens_msg_buf, offset) / 100;
      gps.sacc = XSENS_DATA_RAWGPS_sacc(xsens_msg_buf, offset) / 100;
      gps.pdop = 5;  // FIXME Not output by XSens

      gps_xsens_publish();
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
        ins_ax = XSENS_DATA_Calibrated_accX(xsens_msg_buf, offset);
        ins_ay = XSENS_DATA_Calibrated_accY(xsens_msg_buf, offset);
        ins_az = XSENS_DATA_Calibrated_accZ(xsens_msg_buf, offset);
#if USE_IMU
        imu_xsens.accel_available = TRUE;
#endif
        l++;
      }
      if (!XSENS_MASK_GyrOut(xsens_output_settings)) {
        ins_p = XSENS_DATA_Calibrated_gyrX(xsens_msg_buf, offset);
        ins_q = XSENS_DATA_Calibrated_gyrY(xsens_msg_buf, offset);
        ins_r = XSENS_DATA_Calibrated_gyrZ(xsens_msg_buf, offset);
#if USE_IMU
        imu_xsens.gyro_available = TRUE;
#endif
        l++;
      }
      if (!XSENS_MASK_MagOut(xsens_output_settings)) {
        ins_mx = XSENS_DATA_Calibrated_magX(xsens_msg_buf, offset);
        ins_my = XSENS_DATA_Calibrated_magY(xsens_msg_buf, offset);
        ins_mz = XSENS_DATA_Calibrated_magZ(xsens_msg_buf, offset);
#if USE_IMU
        imu_xsens.mag_available = TRUE;
#endif
        l++;
      }
      offset += l * XSENS_DATA_Calibrated_LENGTH / 3;
    }
    if (XSENS_MASK_Orientation(xsens_output_mode)) {
      if (XSENS_MASK_OrientationMode(xsens_output_settings) == 0x00) {
        float q0 = XSENS_DATA_Quaternion_q0(xsens_msg_buf, offset);
        float q1 = XSENS_DATA_Quaternion_q1(xsens_msg_buf, offset);
        float q2 = XSENS_DATA_Quaternion_q2(xsens_msg_buf, offset);
        float q3 = XSENS_DATA_Quaternion_q3(xsens_msg_buf, offset);
        float dcm00 = 1.0 - 2 * (q2 * q2 + q3 * q3);
        float dcm01 =       2 * (q1 * q2 + q0 * q3);
        float dcm02 =       2 * (q1 * q3 - q0 * q2);
        float dcm12 =       2 * (q2 * q3 + q0 * q1);
        float dcm22 = 1.0 - 2 * (q1 * q1 + q2 * q2);
        ins_phi   = atan2(dcm12, dcm22);
        ins_theta = -asin(dcm02);
        ins_psi   = atan2(dcm01, dcm00);
        offset += XSENS_DATA_Quaternion_LENGTH;
      }
      if (XSENS_MASK_OrientationMode(xsens_output_settings) == 0x01) {
        ins_phi   = XSENS_DATA_Euler_roll(xsens_msg_buf, offset) * M_PI / 180;
        ins_theta = XSENS_DATA_Euler_pitch(xsens_msg_buf, offset) * M_PI / 180;
        ins_psi   = XSENS_DATA_Euler_yaw(xsens_msg_buf, offset) * M_PI / 180;
        offset += XSENS_DATA_Euler_LENGTH;
      }
      if (XSENS_MASK_OrientationMode(xsens_output_settings) == 0x10) {
        offset += XSENS_DATA_Matrix_LENGTH;
      }
      new_ins_attitude = 1;
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
#if (! USE_GPS_XSENS_RAW_DATA) && USE_GPS_XSENS
      lla_f.lat = RadOfDeg(XSENS_DATA_Position_lat(xsens_msg_buf, offset));
      lla_f.lon = RadOfDeg(XSENS_DATA_Position_lon(xsens_msg_buf, offset));
      gps.lla_pos.lat = (int32_t)(DegOfRad(lla_f.lat) * 1e7);
      gps.lla_pos.lon = (int32_t)(DegOfRad(lla_f.lon) * 1e7);
      gps.utm_pos.zone = (DegOfRad(lla_f.lon) + 180) / 6 + 1;
      /* convert to utm */
      utm_of_lla_f(&utm_f, &lla_f);
      /* copy results of utm conversion */
      gps.utm_pos.east = utm_f.east * 100;
      gps.utm_pos.north = utm_f.north * 100;
      ins_x = utm_f.east;
      ins_y = utm_f.north;
      ins_z = XSENS_DATA_Position_alt(xsens_msg_buf, offset); //TODO is this hms or above ellipsoid?
      gps.hmsl = ins_z * 1000;
      // what about gps.lla_pos.alt and gps.utm_pos.alt ?

      gps_xsens_publish();
#endif
      offset += XSENS_DATA_Position_LENGTH;
    }
    if (XSENS_MASK_Velocity(xsens_output_mode)) {
#if (! USE_GPS_XSENS_RAW_DATA) && USE_GPS_XSENS
      ins_vx = XSENS_DATA_Velocity_vx(xsens_msg_buf, offset);
      ins_vy = XSENS_DATA_Velocity_vy(xsens_msg_buf, offset);
      ins_vz = XSENS_DATA_Velocity_vz(xsens_msg_buf, offset);
#endif
      offset += XSENS_DATA_Velocity_LENGTH;
    }
    if (XSENS_MASK_Status(xsens_output_mode)) {
      xsens_msg_status = XSENS_DATA_Status_status(xsens_msg_buf, offset);
#if USE_GPS_XSENS
      if (bit_is_set(xsens_msg_status, 2)) { gps.fix = GPS_FIX_3D; } // gps fix
      else if (bit_is_set(xsens_msg_status, 1)) { gps.fix = 0x01; } // efk valid
      else { gps.fix = GPS_FIX_NONE; }
#ifdef GPS_LED
      if (gps.fix == GPS_FIX_3D) {
        LED_ON(GPS_LED);
      } else {
        LED_TOGGLE(GPS_LED);
      }
#endif // GPS_LED
#endif //  USE_GPS_XSENS
      offset += XSENS_DATA_Status_LENGTH;
    }
    if (XSENS_MASK_TimeStamp(xsens_output_settings)) {
      xsens_time_stamp = XSENS_DATA_TimeStamp_ts(xsens_msg_buf, offset);
#if USE_GPS_XSENS
      gps.tow = xsens_time_stamp;
#endif
      offset += XSENS_DATA_TimeStamp_LENGTH;
    }
    if (XSENS_MASK_UTC(xsens_output_settings)) {
      xsens_time.hour = XSENS_DATA_UTC_hour(xsens_msg_buf, offset);
      xsens_time.min = XSENS_DATA_UTC_min(xsens_msg_buf, offset);
      xsens_time.sec = XSENS_DATA_UTC_sec(xsens_msg_buf, offset);
      xsens_time.nanosec = XSENS_DATA_UTC_nanosec(xsens_msg_buf, offset);
      xsens_time.year = XSENS_DATA_UTC_year(xsens_msg_buf, offset);
      xsens_time.month = XSENS_DATA_UTC_month(xsens_msg_buf, offset);
      xsens_time.day = XSENS_DATA_UTC_day(xsens_msg_buf, offset);

      offset += XSENS_DATA_UTC_LENGTH;
    }
    //}
  }

}


void parse_ins_buffer(uint8_t c)
{
  ck += c;
  switch (xsens_status) {
    case UNINIT:
      if (c != XSENS_START) {
        goto error;
      }
      xsens_status++;
      ck = 0;
      break;
    case GOT_START:
      if (c != XSENS_BID) {
        goto error;
      }
      xsens_status++;
      break;
    case GOT_BID:
      xsens_id = c;
      xsens_status++;
      break;
    case GOT_MID:
      xsens_len = c;
      if (xsens_len > XSENS_MAX_PAYLOAD) {
        goto error;
      }
      xsens_msg_idx = 0;
      xsens_status++;
      break;
    case GOT_LEN:
      xsens_msg_buf[xsens_msg_idx] = c;
      xsens_msg_idx++;
      if (xsens_msg_idx >= xsens_len) {
        xsens_status++;
      }
      break;
    case GOT_DATA:
      if (ck != 0) {
        goto error;
      }
      ins_msg_received = TRUE;
      goto restart;
      break;
    default:
      break;
  }
  return;
error:
restart:
  xsens_status = UNINIT;
  return;
}
