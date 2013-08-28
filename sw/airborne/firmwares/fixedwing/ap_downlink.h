/*
 * Copyright (C) 2006- Pascal Brisset, Antoine Drouin
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

/**
 * @file firmwares/fixedwing/ap_downlink.h
 *
 * Set of macros defining the periodic telemetry messages of AP process.
 *
 * The PeriodicSendAp() macro is generated from the telemetry description
 * (named in conf.xml, usually in conf/telemetry directory). This macro
 * is a sequence of calls to PERIODIC_SEND_message() which have to be defined
 * in the present file.
 *
 */

#ifndef AP_DOWNLINK_H
#define AP_DOWNLINK_H

#include <inttypes.h>

#include "generated/airframe.h"
#include "state.h"

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#include "subsystems/datalink/downlink.h"

#include "messages.h"
#include "generated/periodic_telemetry.h"

// I2C Error counters
#include "mcu_periph/i2c.h"

#if defined DOWNLINK
#define Downlink(x) x
#else
#define Downlink(x) {}
#endif

#define PERIODIC_SEND_ALIVE(_trans, _dev)  DOWNLINK_SEND_ALIVE(_trans, _dev, 16, MD5SUM);

#define PERIODIC_SEND_BAT(_trans, _dev) {                      \
    int16_t amps = (int16_t) (current/10);				\
    Downlink({ int16_t e = energy;                      \
        DOWNLINK_SEND_BAT(_trans, _dev,                        \
                          &v_ctl_throttle_slewed,       \
                          &vsupply,                     \
                          &amps,                        \
                          &autopilot_flight_time,       \
                          &kill_throttle,				\
                          &block_time,					\
                          &stage_time,					\
                          &e);                          \
      });                                               \
  }

#ifdef MCU_SPI_LINK
#define PERIODIC_SEND_DEBUG_MCU_LINK(_trans, _dev) DOWNLINK_SEND_DEBUG_MCU_LINK(_trans, _dev, &link_mcu_nb_err, &link_mcu_fbw_nb_err, &mcu1_ppm_cpt);
#else
#define PERIODIC_SEND_DEBUG_MCU_LINK(_trans, _dev) {}
#endif


#define PERIODIC_SEND_DOWNLINK(_trans, _dev) { \
  static uint16_t last; \
  uint16_t rate = (downlink_nb_bytes - last) / PERIOD_DOWNLINK_Ap_0; \
  last = downlink_nb_bytes; \
  DOWNLINK_SEND_DOWNLINK(_trans, _dev, &downlink_nb_ovrn, &rate, &downlink_nb_msgs); \
}


#define PERIODIC_SEND_ATTITUDE(_trans, _dev) Downlink({ \
    struct FloatEulers* att = stateGetNedToBodyEulers_f(); \
    DOWNLINK_SEND_ATTITUDE(_trans, _dev, &(att->phi), &(att->psi), &(att->theta)); \
})


#ifndef USE_AIRSPEED
#define PERIODIC_SEND_DESIRED(_trans, _dev) { float v_ctl_auto_airspeed_setpoint = NOMINAL_AIRSPEED;  DOWNLINK_SEND_DESIRED(_trans, _dev, &h_ctl_roll_setpoint, &h_ctl_pitch_loop_setpoint, &h_ctl_course_setpoint, &desired_x, &desired_y, &v_ctl_altitude_setpoint, &v_ctl_climb_setpoint, &v_ctl_auto_airspeed_setpoint);}
#else
#define PERIODIC_SEND_DESIRED(_trans, _dev) DOWNLINK_SEND_DESIRED(_trans, _dev, &h_ctl_roll_setpoint, &h_ctl_pitch_loop_setpoint, &h_ctl_course_setpoint, &desired_x, &desired_y, &v_ctl_altitude_setpoint, &v_ctl_climb_setpoint, &v_ctl_auto_airspeed_setpoint);
#endif


#if USE_INFRARED
#define PERIODIC_SEND_STATE_FILTER_STATUS(_trans, _dev) { uint16_t contrast = abs(infrared.roll) + abs(infrared.pitch) + abs(infrared.top); uint8_t mde = 3; if (contrast < 50) mde = 7; DOWNLINK_SEND_STATE_FILTER_STATUS(_trans, _dev, &mde, &contrast); }
#elif USE_IMU && USE_AHRS
#define PERIODIC_SEND_STATE_FILTER_STATUS(_trans, _dev) { uint8_t mde = 3; if (ahrs.status == AHRS_UNINIT) mde = 2; if (ahrs_timeout_counter > 10) mde = 5; uint16_t val = 0; DOWNLINK_SEND_STATE_FILTER_STATUS(_trans, _dev, &mde, &val); }
#else
#define PERIODIC_SEND_STATE_FILTER_STATUS(_trans, _dev) {}
#endif

#define PERIODIC_SEND_NAVIGATION_REF(_trans, _dev)  DOWNLINK_SEND_NAVIGATION_REF(_trans, _dev, &nav_utm_east0, &nav_utm_north0, &nav_utm_zone0);


#define DownlinkSendWp(_trans, _dev, i) {	     \
  float x = nav_utm_east0 +  waypoints[i].x; \
  float y = nav_utm_north0 + waypoints[i].y; \
  DOWNLINK_SEND_WP_MOVED(_trans, _dev, &i, &x, &y, &(waypoints[i].a),&nav_utm_zone0); \
}


#define PERIODIC_SEND_WP_MOVED(_trans, _dev) { \
  static uint8_t i; \
  i++; if (i >= nb_waypoint) i = 0; \
  DownlinkSendWp(_trans, _dev, i);	    \
}

#if defined RADIO_CALIB && defined RADIO_CONTROL_SETTINGS
#include "rc_settings.h"
#define PERIODIC_SEND_PPRZ_MODE(_trans, _dev) DOWNLINK_SEND_PPRZ_MODE(_trans, _dev, &pprz_mode, &v_ctl_mode, &lateral_mode, &horizontal_mode, &rc_settings_mode, &mcu1_status);
#define PERIODIC_SEND_SETTINGS(_trans, _dev) if (!RcSettingsOff()) DOWNLINK_SEND_SETTINGS(_trans, _dev, &slider_1_val, &slider_2_val);
#else
#define PERIODIC_SEND_PPRZ_MODE(_trans, _dev) {                         \
    uint8_t rc_settings_mode_none = 0;                                  \
    DOWNLINK_SEND_PPRZ_MODE(_trans, _dev, &pprz_mode, &v_ctl_mode, &lateral_mode, &horizontal_mode, &rc_settings_mode_none, &mcu1_status); \
  }
#define PERIODIC_SEND_SETTINGS(_trans, _dev) {}
#endif

#if USE_INFRARED || USE_INFRARED_TELEMETRY
#include "subsystems/sensors/infrared.h"
#define PERIODIC_SEND_IR_SENSORS(_trans, _dev) DOWNLINK_SEND_IR_SENSORS(_trans, _dev, &infrared.value.ir1, &infrared.value.ir2, &infrared.pitch, &infrared.roll, &infrared.top);
#else
#define PERIODIC_SEND_IR_SENSORS(_trans, _dev) ;
#endif

#define PERIODIC_SEND_ADC(_trans, _dev) {}


#define PERIODIC_SEND_CALIBRATION(_trans, _dev) DOWNLINK_SEND_CALIBRATION(_trans, _dev, &v_ctl_auto_throttle_sum_err, &v_ctl_auto_throttle_submode)

#define PERIODIC_SEND_CIRCLE(_trans, _dev) if (nav_in_circle) { DOWNLINK_SEND_CIRCLE(_trans, _dev, &nav_circle_x, &nav_circle_y, &nav_circle_radius); }

#define PERIODIC_SEND_SEGMENT(_trans, _dev) if (nav_in_segment) { DOWNLINK_SEND_SEGMENT(_trans, _dev, &nav_segment_x_1, &nav_segment_y_1, &nav_segment_x_2, &nav_segment_y_2); }

#if USE_IMU_FLOAT
#  include "subsystems/imu.h"
#  define PERIODIC_SEND_IMU_ACCEL(_trans, _dev) { DOWNLINK_SEND_IMU_ACCEL(_trans, _dev, &imuf.accel.x, &imuf.accel.y, &imuf.accel.z)}
#  define PERIODIC_SEND_IMU_GYRO(_trans, _dev) { DOWNLINK_SEND_IMU_GYRO(_trans, _dev, &imuf.gyro.p, &imuf.gyro.q, &imuf.gyro.r)}
#else
#ifdef IMU_TYPE_H
#  ifdef INS_MODULE_H
#  include "modules/ins/ins_module.h"
#    define PERIODIC_SEND_IMU_ACCEL_RAW(_trans, _dev) {}
#    define PERIODIC_SEND_IMU_GYRO_RAW(_trans, _dev) {}
#    define PERIODIC_SEND_IMU_MAG_RAW(_trans, _dev) {}
#    define PERIODIC_SEND_IMU_GYRO(_trans, _dev) { DOWNLINK_SEND_IMU_GYRO(_trans, _dev, &ins_p, &ins_q, &ins_r)}
#    define PERIODIC_SEND_IMU_ACCEL(_trans, _dev) { DOWNLINK_SEND_IMU_ACCEL(_trans, _dev, &ins_ax, &ins_ay, &ins_az)}
#    define PERIODIC_SEND_IMU_MAG(_trans, _dev) { DOWNLINK_SEND_IMU_MAG(_trans, _dev, &ins_mx, &ins_my, &ins_mz)}
#  else
#    include "subsystems/imu.h"
#    define PERIODIC_SEND_IMU_ACCEL_RAW(_trans, _dev) { DOWNLINK_SEND_IMU_ACCEL_RAW(_trans, _dev, &imu.accel_unscaled.x, &imu.accel_unscaled.y, &imu.accel_unscaled.z)}
#    define PERIODIC_SEND_IMU_GYRO_RAW(_trans, _dev) { DOWNLINK_SEND_IMU_GYRO_RAW(_trans, _dev, &imu.gyro_unscaled.p, &imu.gyro_unscaled.q, &imu.gyro_unscaled.r)}
#    define PERIODIC_SEND_IMU_MAG_RAW(_trans, _dev) { DOWNLINK_SEND_IMU_MAG_RAW(_trans, _dev, &imu.mag_unscaled.x, &imu.mag_unscaled.y, &imu.mag_unscaled.z)}
#    define PERIODIC_SEND_IMU_ACCEL(_trans, _dev) { struct FloatVect3 accel_float; ACCELS_FLOAT_OF_BFP(accel_float, imu.accel); DOWNLINK_SEND_IMU_ACCEL(_trans, _dev, &accel_float.x, &accel_float.y, &accel_float.z)}
#    define PERIODIC_SEND_IMU_GYRO(_trans, _dev) { struct FloatRates gyro_float; RATES_FLOAT_OF_BFP(gyro_float, imu.gyro); DOWNLINK_SEND_IMU_GYRO(_trans, _dev, &gyro_float.p, &gyro_float.q, &gyro_float.r)}
#    ifdef USE_MAGNETOMETER
#      define PERIODIC_SEND_IMU_MAG(_trans, _dev) { struct FloatVect3 mag_float; MAGS_FLOAT_OF_BFP(mag_float, imu.mag); DOWNLINK_SEND_IMU_MAG(_trans, _dev, &mag_float.x, &mag_float.y, &mag_float.z)}
#    else
#      define PERIODIC_SEND_IMU_MAG(_trans, _dev) {}
#    endif
#  endif
#else
#    define PERIODIC_SEND_IMU_ACCEL_RAW(_trans, _dev) {}
#    define PERIODIC_SEND_IMU_GYRO_RAW(_trans, _dev) {}
#    define PERIODIC_SEND_IMU_MAG_RAW(_trans, _dev) {}
#    define PERIODIC_SEND_IMU_ACCEL(_trans, _dev) {}
#    define PERIODIC_SEND_IMU_GYRO(_trans, _dev) {}
#    define PERIODIC_SEND_IMU_MAG(_trans, _dev) {}
#endif
#endif

#ifdef IMU_ANALOG
#define PERIODIC_SEND_IMU(_trans, _dev) { int16_t dummy = 42; DOWNLINK_SEND_IMU(_trans, _dev, &(from_fbw.euler_dot[0]), &(from_fbw.euler_dot[1]), &(from_fbw.euler_dot[2]), &dummy, &dummy, &dummy); }
#else
#define PERIODIC_SEND_IMU(_trans, _dev) {}
#endif

#define PERIODIC_SEND_ESTIMATOR(_trans, _dev) DOWNLINK_SEND_ESTIMATOR(_trans, _dev, &(stateGetPositionUtm_f()->alt), &(stateGetSpeedEnu_f()->z))

#define SEND_NAVIGATION(_trans, _dev) Downlink({ \
    uint8_t _circle_count = NavCircleCount(); \
    struct EnuCoor_f* pos = stateGetPositionEnu_f(); \
    DOWNLINK_SEND_NAVIGATION(_trans, _dev, &nav_block, &nav_stage, &(pos->x), &(pos->y), &dist2_to_wp, &dist2_to_home, &_circle_count, &nav_oval_count); \
    })

#define PERIODIC_SEND_NAVIGATION(_trans, _dev) SEND_NAVIGATION(_trans, _dev)


#if defined CAM || defined MOBILE_CAM
#define SEND_CAM(_trans, _dev) Downlink({ int16_t x = cam_target_x; int16_t y = cam_target_y; int16_t phi = DegOfRad(cam_phi_c); int16_t theta = DegOfRad(cam_theta_c); DOWNLINK_SEND_CAM(_trans, _dev, &phi, &theta, &x, &y);})
#define PERIODIC_SEND_CAM_POINT(_trans, _dev) DOWNLINK_SEND_CAM_POINT(_trans, _dev, &cam_point_distance_from_home, &cam_point_lat, &cam_point_lon)
#else
#define SEND_CAM(_trans, _dev) {}
#define PERIODIC_SEND_CAM_POINT(_trans, _dev) {}
#endif

#define PERIODIC_SEND_DL_VALUE(_trans, _dev) PeriodicSendDlValue(_trans, _dev) /** generated from the xml settings config in conf/settings */

#define PERIODIC_SEND_SURVEY(_trans, _dev) { \
  if (nav_survey_active) \
    DOWNLINK_SEND_SURVEY(_trans, _dev, &nav_survey_east, &nav_survey_north, &nav_survey_west, &nav_survey_south); \
  }

#define PERIODIC_SEND_RANGEFINDER(_trans, _dev) DOWNLINK_SEND_RANGEFINDER(_trans, _dev, &rangemeter, &ctl_grz_z_dot, &ctl_grz_z_dot_sum_err, &ctl_grz_z_dot_setpoint, &ctl_grz_z_sum_err, &ctl_grz_z_setpoint, &flying)

#define PERIODIC_SEND_TUNE_ROLL(_trans, _dev) DOWNLINK_SEND_TUNE_ROLL(_trans, _dev, &(stateGetBodyRates_f()->p), &(stateGetNedToBodyEulers_f()->phi), &h_ctl_roll_setpoint);

#if USE_GPS || defined SITL
#define PERIODIC_SEND_GPS_SOL(_trans, _dev) DOWNLINK_SEND_GPS_SOL(_trans, _dev, &gps.pacc, &gps.sacc, &gps.pdop, &gps.num_sv)
#else
#define PERIODIC_SEND_GPS_SOL(_trans, _dev) {}
#endif

#define PERIODIC_SEND_GPS(_trans, _dev) {                                      \
    static uint8_t i;                                                   \
    int16_t climb = -gps.ned_vel.z;                                     \
    int16_t course = (DegOfRad(gps.course)/((int32_t)1e6));             \
    DOWNLINK_SEND_GPS(_trans, _dev, &gps.fix, &gps.utm_pos.east, &gps.utm_pos.north, &course, &gps.hmsl, &gps.gspeed, &climb, &gps.week, &gps.tow, &gps.utm_pos.zone, &i); \
    if ((gps.fix != GPS_FIX_3D) && (i >= gps.nb_channels)) i = 0;                                    \
    if (i >= gps.nb_channels * 2) i = 0;                                    \
    if (i < gps.nb_channels && ((gps.fix != GPS_FIX_3D) || (gps.svinfos[i].cno > 0))) { \
      DOWNLINK_SEND_SVINFO(_trans, _dev, &i, &gps.svinfos[i].svid, &gps.svinfos[i].flags, &gps.svinfos[i].qi, &gps.svinfos[i].cno, &gps.svinfos[i].elev, &gps.svinfos[i].azim); \
    }                                                                   \
    i++;                                                                \
}

#if USE_GPS
#define PERIODIC_SEND_GPS_INT(_trans, _dev) {   \
  DOWNLINK_SEND_GPS_INT( _trans, _dev,          \
                         &gps.ecef_pos.x,       \
                         &gps.ecef_pos.y,       \
                         &gps.ecef_pos.z,       \
                         &gps.lla_pos.lat,      \
                         &gps.lla_pos.lon,      \
                         &gps.lla_pos.alt,      \
                         &gps.hmsl,             \
                         &gps.ecef_vel.x,       \
                         &gps.ecef_vel.y,       \
                         &gps.ecef_vel.z,       \
                         &gps.pacc,             \
                         &gps.sacc,             \
                         &gps.tow,              \
                         &gps.pdop,             \
                         &gps.num_sv,           \
                         &gps.fix);             \
  }

#define PERIODIC_SEND_GPS_LLA(_trans, _dev) {               \
    uint8_t err = 0;                                        \
    int16_t climb = -gps.ned_vel.z;                         \
    int16_t course = (DegOfRad(gps.course)/((int32_t)1e6)); \
    DOWNLINK_SEND_GPS_LLA( _trans, _dev,                    \
                           &gps.lla_pos.lat,                \
                           &gps.lla_pos.lon,                \
                           &gps.lla_pos.alt,                \
                           &course,                         \
                           &gps.gspeed,                     \
                           &climb,                          \
                           &gps.week,                       \
                           &gps.tow,                        \
                           &gps.fix,                        \
                           &err);                           \
  }
#endif

#if USE_BARO_MS5534A
//#include "baro_MS5534A.h"
#define PERIODIC_SEND_BARO_MS5534A(_trans, _dev) DOWNLINK_SEND_BARO_MS5534A(_trans, _dev, &baro_MS5534A_pressure, &baro_MS5534A_temp, &baro_MS5534A_z)
#else
#define PERIODIC_SEND_BARO_MS5534A(_trans, _dev) {}
#endif

#if USE_BARO_SCP
#include "baro_scp.h"
#define PERIODIC_SEND_SCP_STATUS(_trans, _dev) DOWNLINK_SEND_SCP_STATUS(_trans, _dev, &baro_scp_pressure, &baro_scp_temperature)
#else
#define PERIODIC_SEND_SCP_STATUS(_trans, _dev) {}
#endif

#include "subsystems/air_data.h"
#define PERIODIC_SEND_BARO_RAW(_trans, _dev) {      \
    DOWNLINK_SEND_BARO_RAW(_trans, _dev,            \
                           &air_data.pressure,      \
                           &air_data.differential); \
  }

#ifdef MEASURE_AIRSPEED
#define PERIODIC_SEND_AIRSPEED(_trans, _dev) { \
  float* airspeed = stateGetAirspeed_f(); \
  DOWNLINK_SEND_AIRSPEED (_trans, _dev, airspeed, airspeed, airspeed, airspeed); \
}
#elif USE_AIRSPEED
#define PERIODIC_SEND_AIRSPEED(_trans, _dev) DOWNLINK_SEND_AIRSPEED (_trans, _dev, stateGetAirspeed_f(), &v_ctl_auto_airspeed_setpoint, &v_ctl_auto_airspeed_controlled, &v_ctl_auto_groundspeed_setpoint)
#else
#define PERIODIC_SEND_AIRSPEED(_trans, _dev) {}
#endif

#define PERIODIC_SEND_ENERGY(_trans, _dev) Downlink({ const int16_t e = energy; const float vsup = ((float)vsupply) / 10.0f; const float curs = ((float) current)/1000.0f;  const float power = vsup * curs; DOWNLINK_SEND_ENERGY(_trans, _dev, &vsup, &curs, &e, &power); })


#include "firmwares/fixedwing/stabilization/stabilization_adaptive.h"
#define PERIODIC_SEND_H_CTL_A(_trans, _dev) DOWNLINK_SEND_H_CTL_A(_trans, _dev, &h_ctl_roll_sum_err, &h_ctl_ref_roll_angle, &h_ctl_pitch_sum_err, &h_ctl_ref_pitch_angle)

#ifdef USE_GX3
#define PERIODIC_SEND_GX3_INFO(_trans, _dev) DOWNLINK_SEND_GX3_INFO(_trans, _dev,\
    &ahrs_impl.GX3_freq,			\
    &ahrs_impl.GX3_packet.chksm_error,	\
    &ahrs_impl.GX3_packet.hdr_error,	\
    &ahrs_impl.GX3_chksm)
#else
#define PERIODIC_SEND_GX3_INFO(_trans, _dev) {}
#endif

#ifdef USE_I2C0
#define PERIODIC_SEND_I2C0_ERRORS(_trans, _dev) {                       \
    uint16_t i2c0_queue_full_cnt        = i2c0.errors->queue_full_cnt;  \
    uint16_t i2c0_ack_fail_cnt          = i2c0.errors->ack_fail_cnt;    \
    uint16_t i2c0_miss_start_stop_cnt   = i2c0.errors->miss_start_stop_cnt; \
    uint16_t i2c0_arb_lost_cnt          = i2c0.errors->arb_lost_cnt;    \
    uint16_t i2c0_over_under_cnt        = i2c0.errors->over_under_cnt;  \
    uint16_t i2c0_pec_recep_cnt         = i2c0.errors->pec_recep_cnt;   \
    uint16_t i2c0_timeout_tlow_cnt      = i2c0.errors->timeout_tlow_cnt; \
    uint16_t i2c0_smbus_alert_cnt       = i2c0.errors->smbus_alert_cnt; \
    uint16_t i2c0_unexpected_event_cnt  = i2c0.errors->unexpected_event_cnt; \
    uint32_t i2c0_last_unexpected_event = i2c0.errors->last_unexpected_event; \
    const uint8_t _bus0 = 0;                                            \
    DOWNLINK_SEND_I2C_ERRORS(_trans, _dev,                              \
                             &i2c0_queue_full_cnt,                      \
                             &i2c0_ack_fail_cnt,                        \
                             &i2c0_miss_start_stop_cnt,                 \
                             &i2c0_arb_lost_cnt,                        \
                             &i2c0_over_under_cnt,                      \
                             &i2c0_pec_recep_cnt,                       \
                             &i2c0_timeout_tlow_cnt,                    \
                             &i2c0_smbus_alert_cnt,                     \
                             &i2c0_unexpected_event_cnt,                \
                             &i2c0_last_unexpected_event,               \
                             &_bus0);                                   \
  }
#else
#define PERIODIC_SEND_I2C0_ERRORS(_trans, _dev) {}
#endif

#ifdef USE_I2C1
#define PERIODIC_SEND_I2C1_ERRORS(_trans, _dev) {                       \
    uint16_t i2c1_queue_full_cnt        = i2c1.errors->queue_full_cnt;  \
    uint16_t i2c1_ack_fail_cnt          = i2c1.errors->ack_fail_cnt;    \
    uint16_t i2c1_miss_start_stop_cnt   = i2c1.errors->miss_start_stop_cnt; \
    uint16_t i2c1_arb_lost_cnt          = i2c1.errors->arb_lost_cnt;    \
    uint16_t i2c1_over_under_cnt        = i2c1.errors->over_under_cnt;  \
    uint16_t i2c1_pec_recep_cnt         = i2c1.errors->pec_recep_cnt;   \
    uint16_t i2c1_timeout_tlow_cnt      = i2c1.errors->timeout_tlow_cnt; \
    uint16_t i2c1_smbus_alert_cnt       = i2c1.errors->smbus_alert_cnt; \
    uint16_t i2c1_unexpected_event_cnt  = i2c1.errors->unexpected_event_cnt; \
    uint32_t i2c1_last_unexpected_event = i2c1.errors->last_unexpected_event; \
    const uint8_t _bus1 = 1;                                            \
    DOWNLINK_SEND_I2C_ERRORS(_trans, _dev,                              \
                             &i2c1_queue_full_cnt,                      \
                             &i2c1_ack_fail_cnt,                        \
                             &i2c1_miss_start_stop_cnt,                 \
                             &i2c1_arb_lost_cnt,                        \
                             &i2c1_over_under_cnt,                      \
                             &i2c1_pec_recep_cnt,                       \
                             &i2c1_timeout_tlow_cnt,                    \
                             &i2c1_smbus_alert_cnt,                     \
                             &i2c1_unexpected_event_cnt,                \
                             &i2c1_last_unexpected_event,               \
                             &_bus1);                                   \
  }
#else
#define PERIODIC_SEND_I2C1_ERRORS(_trans, _dev) {}
#endif

#ifdef USE_I2C2
#define PERIODIC_SEND_I2C2_ERRORS(_trans, _dev) {                       \
    uint16_t i2c2_queue_full_cnt        = i2c2.errors->queue_full_cnt;  \
    uint16_t i2c2_ack_fail_cnt          = i2c2.errors->ack_fail_cnt;    \
    uint16_t i2c2_miss_start_stop_cnt   = i2c2.errors->miss_start_stop_cnt; \
    uint16_t i2c2_arb_lost_cnt          = i2c2.errors->arb_lost_cnt;    \
    uint16_t i2c2_over_under_cnt        = i2c2.errors->over_under_cnt;  \
    uint16_t i2c2_pec_recep_cnt         = i2c2.errors->pec_recep_cnt;   \
    uint16_t i2c2_timeout_tlow_cnt      = i2c2.errors->timeout_tlow_cnt; \
    uint16_t i2c2_smbus_alert_cnt       = i2c2.errors->smbus_alert_cnt; \
    uint16_t i2c2_unexpected_event_cnt  = i2c2.errors->unexpected_event_cnt; \
    uint32_t i2c2_last_unexpected_event = i2c2.errors->last_unexpected_event; \
    const uint8_t _bus2 = 2;                                            \
    DOWNLINK_SEND_I2C_ERRORS(_trans, _dev,                              \
                             &i2c2_queue_full_cnt,                      \
                             &i2c2_ack_fail_cnt,                        \
                             &i2c2_miss_start_stop_cnt,                 \
                             &i2c2_arb_lost_cnt,                        \
                             &i2c2_over_under_cnt,                      \
                             &i2c2_pec_recep_cnt,                       \
                             &i2c2_timeout_tlow_cnt,                    \
                             &i2c2_smbus_alert_cnt,                     \
                             &i2c2_unexpected_event_cnt,                \
                             &i2c2_last_unexpected_event,               \
                             &_bus2);                                   \
  }
#else
#define PERIODIC_SEND_I2C2_ERRORS(_trans, _dev) {}
#endif

#ifdef USE_I2C3
#define PERIODIC_SEND_I2C3_ERRORS(_trans, _dev) {                       \
    uint16_t i2c3_queue_full_cnt        = i2c3.errors->queue_full_cnt;  \
    uint16_t i2c3_ack_fail_cnt          = i2c3.errors->ack_fail_cnt;    \
    uint16_t i2c3_miss_start_stop_cnt   = i2c3.errors->miss_start_stop_cnt; \
    uint16_t i2c3_arb_lost_cnt          = i2c3.errors->arb_lost_cnt;    \
    uint16_t i2c3_over_under_cnt        = i2c3.errors->over_under_cnt;  \
    uint16_t i2c3_pec_recep_cnt         = i2c3.errors->pec_recep_cnt;   \
    uint16_t i2c3_timeout_tlow_cnt      = i2c3.errors->timeout_tlow_cnt; \
    uint16_t i2c3_smbus_alert_cnt       = i2c3.errors->smbus_alert_cnt; \
    uint16_t i2c3_unexpected_event_cnt  = i2c3.errors->unexpected_event_cnt; \
    uint32_t i2c3_last_unexpected_event = i2c3.errors->last_unexpected_event; \
    const uint8_t _bus3 = 3;                                            \
    DOWNLINK_SEND_I2C_ERRORS(_trans, _dev,                              \
                             &i2c3_queue_full_cnt,                      \
                             &i2c3_ack_fail_cnt,                        \
                             &i2c3_miss_start_stop_cnt,                 \
                             &i2c3_arb_lost_cnt,                        \
                             &i2c3_over_under_cnt,                      \
                             &i2c3_pec_recep_cnt,                       \
                             &i2c3_timeout_tlow_cnt,                    \
                             &i2c3_smbus_alert_cnt,                     \
                             &i2c3_unexpected_event_cnt,                \
                             &i2c3_last_unexpected_event,               \
                             &_bus3);                                   \
  }
#else
#define PERIODIC_SEND_I2C3_ERRORS(_trans, _dev) {}
#endif

#ifndef SITL
#define PERIODIC_SEND_I2C_ERRORS(_trans, _dev) {        \
    static uint8_t _i2c_nb_cnt = 0;                     \
    switch (_i2c_nb_cnt) {                              \
      case 0:                                           \
        PERIODIC_SEND_I2C0_ERRORS(_trans, _dev); break; \
      case 1:                                           \
        PERIODIC_SEND_I2C1_ERRORS(_trans, _dev); break; \
      case 2:                                           \
        PERIODIC_SEND_I2C2_ERRORS(_trans, _dev); break; \
      case 3:                                           \
        PERIODIC_SEND_I2C3_ERRORS(_trans, _dev); break; \
      default:                                          \
        break;                                          \
    }                                                   \
    _i2c_nb_cnt++;                                      \
    if (_i2c_nb_cnt == 3)                               \
      _i2c_nb_cnt = 0;                                  \
  }
#else
#define PERIODIC_SEND_I2C_ERRORS(_trans, _dev) {}
#endif

#endif /* AP_DOWNLINK_H */
