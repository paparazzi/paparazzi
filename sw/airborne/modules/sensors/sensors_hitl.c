/*
 * Copyright (C) 2023 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "modules/sensors/sensors_hitl.h"
#include "modules/imu/imu.h"
#include "modules/gps/gps.h"
#include "modules/core/abi.h"
#include "modules/energy/electrical.h"
#include "generated/airframe.h"
#include "modules/datalink/datalink.h"
#include "modules/datalink/telemetry.h"
#include "nps_sensors_params_common.h"
#if USE_BATTERY_MONITOR
#include "modules/energy/electrical.h"
#endif

struct ImuHitl {
  uint8_t mag_available;
  uint8_t accel_available;
  uint8_t gyro_available;

  struct Int32Rates gyro;
  struct Int32Vect3 accel;
  struct Int32Vect3 mag;
};

struct ImuHitl imu_hitl;
struct GpsState gps_hitl;
bool gps_has_fix;

static bool sensors_hitl_msg_available = false;
static uint8_t sensors_hitl_dl_buffer[MSG_SIZE]  __attribute__((aligned));
static struct pprz_transport sensors_hitl_tp;

void sensors_hitl_init(void)
{
  pprz_transport_init(&sensors_hitl_tp); // for receiving only

  gps_has_fix = true;

  imu_hitl.gyro_available = false;
  imu_hitl.mag_available = false;
  imu_hitl.accel_available = false;

  // Set the default scaling
  const struct Int32Rates gyro_scale[2] = {
    {NPS_GYRO_SENSITIVITY_PP_NUM, NPS_GYRO_SENSITIVITY_QQ_NUM, NPS_GYRO_SENSITIVITY_RR_NUM},
    {NPS_GYRO_SENSITIVITY_PP_DEN, NPS_GYRO_SENSITIVITY_QQ_DEN, NPS_GYRO_SENSITIVITY_RR_DEN}
  };
  const struct Int32Rates gyro_neutral = {
    NPS_GYRO_NEUTRAL_P, NPS_GYRO_NEUTRAL_Q, NPS_GYRO_NEUTRAL_R
  };
  const struct Int32Vect3 accel_scale[2] = {
    {NPS_ACCEL_SENSITIVITY_XX_NUM, NPS_ACCEL_SENSITIVITY_YY_NUM, NPS_ACCEL_SENSITIVITY_ZZ_NUM},
    {NPS_ACCEL_SENSITIVITY_XX_DEN, NPS_ACCEL_SENSITIVITY_YY_DEN, NPS_ACCEL_SENSITIVITY_ZZ_DEN}
  };
  const struct Int32Vect3 accel_neutral = {
    NPS_ACCEL_NEUTRAL_X, NPS_ACCEL_NEUTRAL_Y, NPS_ACCEL_NEUTRAL_Z
  };
  const struct Int32Vect3 mag_scale[2] = {
    {NPS_MAG_SENSITIVITY_XX_NUM, NPS_MAG_SENSITIVITY_YY_NUM, NPS_MAG_SENSITIVITY_ZZ_NUM},
    {NPS_MAG_SENSITIVITY_XX_DEN, NPS_MAG_SENSITIVITY_YY_DEN, NPS_MAG_SENSITIVITY_ZZ_DEN}
  };
  const struct Int32Vect3 mag_neutral = {
    NPS_MAG_NEUTRAL_X, NPS_MAG_NEUTRAL_Y, NPS_MAG_NEUTRAL_Z
  };
  imu_set_defaults_gyro(IMU_NPS_ID, NULL, &gyro_neutral, gyro_scale);
  imu_set_defaults_accel(IMU_NPS_ID, NULL, &accel_neutral, accel_scale);
  imu_set_defaults_mag(IMU_NPS_ID, NULL, &mag_neutral, mag_scale);
}

void sensors_hitl_periodic(void) {
#if USE_BATTERY_MONITOR
#ifdef MAX_BAT_LEVEL
  // init vsupply to MAX_BAT in case battery voltage is not available
  electrical.vsupply = MAX_BAT_LEVEL;
#else
  electrical.vsupply = 12.f; // 3S battery
#endif
#endif
}

void sensors_hitl_parse_HITL_IMU(uint8_t *buf)
{
  if (DL_HITL_IMU_ac_id(buf) != AC_ID) {
    return;
  }

  RATES_ASSIGN(imu_hitl.gyro,
      NPS_GYRO_SIGN_P * DL_HITL_IMU_gp(buf),
      NPS_GYRO_SIGN_Q * DL_HITL_IMU_gq(buf),
      NPS_GYRO_SIGN_R * DL_HITL_IMU_gr(buf));
  VECT3_ASSIGN(imu_hitl.accel,
      NPS_ACCEL_SIGN_X * DL_HITL_IMU_ax(buf),
      NPS_ACCEL_SIGN_Y * DL_HITL_IMU_ay(buf),
      NPS_ACCEL_SIGN_Z * DL_HITL_IMU_az(buf));
  VECT3_ASSIGN(imu_hitl.mag,
      NPS_MAG_SIGN_X * DL_HITL_IMU_mx(buf),
      NPS_MAG_SIGN_Y * DL_HITL_IMU_my(buf),
      NPS_MAG_SIGN_Z * DL_HITL_IMU_mz(buf));

  imu_hitl.accel_available = true;
  imu_hitl.gyro_available = true;
  imu_hitl.mag_available = true;
}

void sensors_hitl_parse_HITL_GPS(uint8_t *buf)
{
  if (DL_HITL_GPS_ac_id(buf) != AC_ID) {
    return;
  }

  gps_hitl.week = 1794;
  gps_hitl.tow = DL_HITL_GPS_time(buf) * 1000;

  gps_hitl.ecef_vel.x = DL_HITL_GPS_ecef_vel_x(buf) * 100.;
  gps_hitl.ecef_vel.y = DL_HITL_GPS_ecef_vel_y(buf) * 100.;
  gps_hitl.ecef_vel.z = DL_HITL_GPS_ecef_vel_z(buf) * 100.;
  SetBit(gps_hitl.valid_fields, GPS_VALID_VEL_ECEF_BIT);

  gps_hitl.lla_pos.lat = DL_HITL_GPS_lat(buf) * 1e7;
  gps_hitl.lla_pos.lon = DL_HITL_GPS_lon(buf) * 1e7;
  gps_hitl.lla_pos.alt = DL_HITL_GPS_alt(buf) * 1000.;
  SetBit(gps_hitl.valid_fields, GPS_VALID_POS_LLA_BIT);

  gps_hitl.hmsl        = DL_HITL_GPS_hmsl(buf) * 1000.;
  SetBit(gps_hitl.valid_fields, GPS_VALID_HMSL_BIT);

  /* calc NED speed from ECEF */
  struct LtpDef_i ref_ltp;
  ltp_def_from_lla_i(&ref_ltp, &gps_hitl.lla_pos);
  struct NedCoor_i ned_vel_i;
  ned_of_ecef_vect_i(&ned_vel_i, &ref_ltp, &gps_hitl.ecef_vel);
  gps_hitl.ned_vel.x = ned_vel_i.x;
  gps_hitl.ned_vel.y = ned_vel_i.y;
  gps_hitl.ned_vel.z = ned_vel_i.z;
  SetBit(gps_hitl.valid_fields, GPS_VALID_VEL_NED_BIT);
  struct NedCoor_f ned_vel_f;
  VECT3_FLOAT_OF_CM(ned_vel_f, gps_hitl.ned_vel);

  /* horizontal and 3d ground speed in cm/s */
  gps_hitl.gspeed = sqrtf(ned_vel_f.x * ned_vel_f.x + ned_vel_f.y * ned_vel_f.y) * 100;
  gps_hitl.speed_3d = sqrtf(ned_vel_f.x * ned_vel_f.x + ned_vel_f.y * ned_vel_f.y + ned_vel_f.z * ned_vel_f.z) * 100;

  /* ground course in radians * 1e7 */
  gps_hitl.course = atan2f(ned_vel_f.y, ned_vel_f.x) * 1e7;
  SetBit(gps_hitl.valid_fields, GPS_VALID_COURSE_BIT);

  gps_hitl.pacc = 650;
  gps_hitl.hacc = 450;
  gps_hitl.vacc = 200;
  gps_hitl.sacc = 100;
  gps_hitl.pdop = 650;

  if (gps_has_fix) {
    gps_hitl.num_sv = 11;
    gps_hitl.fix = GPS_FIX_3D;
  } else {
    gps_hitl.num_sv = 1;
    gps_hitl.fix = GPS_FIX_NONE;
  }

  // publish gps data
  uint32_t now_ts = get_sys_time_usec();
  gps_hitl.last_msg_ticks = sys_time.nb_sec_rem;
  gps_hitl.last_msg_time = sys_time.nb_sec;
  if (gps_hitl.fix == GPS_FIX_3D) {
    gps_hitl.last_3dfix_ticks = sys_time.nb_sec_rem;
    gps_hitl.last_3dfix_time = sys_time.nb_sec;
  }
  AbiSendMsgGPS(GPS_SIM_ID, now_ts, &gps_hitl);
}

void sensors_hitl_parse_HITL_AIR_DATA(uint8_t *buf) {
  if (DL_HITL_AIR_DATA_ac_id(buf) != AC_ID) {
    return;
  }

  uint8_t flag = DL_HITL_AIR_DATA_update_flag(buf);
  if (bit_is_set(flag, 0)) {
    uint32_t now_ts = get_sys_time_usec();
    float pressure = DL_HITL_AIR_DATA_baro(buf);
    AbiSendMsgBARO_ABS(BARO_SIM_SENDER_ID, now_ts, pressure);
  }
  if (bit_is_set(flag, 1)) {
    AbiSendMsgAIRSPEED(AIRSPEED_NPS_ID, DL_HITL_AIR_DATA_airspeed(buf));
  }
  if (bit_is_set(flag, 2) || bit_is_set(flag, 3)) {
    uint8_t incidence_flag = (flag >> 2) & (0x3);
    float aoa = DL_HITL_AIR_DATA_aoa(buf);
    float sideslip = DL_HITL_AIR_DATA_sideslip(buf);
    AbiSendMsgINCIDENCE(INCIDENCE_NPS_ID, incidence_flag, aoa, sideslip);
  }
}


void sensors_hitl_event(void)
{
  uint32_t now_ts = get_sys_time_usec();
  if (imu_hitl.gyro_available) {
    AbiSendMsgIMU_GYRO_RAW(IMU_NPS_ID, now_ts, &imu_hitl.gyro, 1, NPS_PROPAGATE, NAN);
    imu_hitl.gyro_available = false;
  }
  if (imu_hitl.accel_available) {
    AbiSendMsgIMU_ACCEL_RAW(IMU_NPS_ID, now_ts, &imu_hitl.accel, 1, NPS_PROPAGATE, NAN);
    imu_hitl.accel_available = false;
  }
  if (imu_hitl.mag_available) {
    AbiSendMsgIMU_MAG_RAW(IMU_NPS_ID, now_ts, &imu_hitl.mag);
    imu_hitl.mag_available = false;
  }

  // parse incoming messages
  pprz_check_and_parse(&HITL_DEVICE.device, &sensors_hitl_tp, sensors_hitl_dl_buffer, &sensors_hitl_msg_available);
  DlCheckAndParse(&HITL_DEVICE.device, &sensors_hitl_tp.trans_tx, sensors_hitl_dl_buffer, &sensors_hitl_msg_available, false);
}

void imu_feed_gyro_accel(void) {}
void imu_feed_mag(void) {}
void gps_feed_value(void) {}

