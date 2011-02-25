/*
 * Paparazzi $Id$
 *
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
 *
 */

/** \file ap_downlink.h
 *  \brief Set of macros defining the periodic telemetry messages of AP process
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

#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#include "downlink.h"

#include "messages.h"
#include "generated/periodic.h"

//#include "generated/modules.h"

#if defined DOWNLINK
#define Downlink(x) x
#else
#define Downlink(x) {}
#endif


#define PERIODIC_SEND_ALIVE(_chan)  DOWNLINK_SEND_ALIVE(_chan, 16, MD5SUM);

#define PERIODIC_SEND_BAT(_chan) { \
	uint16_t zero = 0; \
	Downlink({ int16_t e = energy; \
			   DOWNLINK_SEND_BAT(_chan, \
								 &v_ctl_throttle_slewed, \
								 &vsupply, \
				   				 &zero, \
								 &estimator_flight_time, \
								 &kill_throttle, \
								 &block_time, \
								 &stage_time, \
								 &e); \
	}); \
}

#ifdef MCU_SPI_LINK
#define PERIODIC_SEND_DEBUG_MCU_LINK(_chan) DOWNLINK_SEND_DEBUG_MCU_LINK(_chan, &link_mcu_nb_err, &link_mcu_fbw_nb_err, &mcu1_ppm_cpt);
#else
#define PERIODIC_SEND_DEBUG_MCU_LINK(_chan) {}
#endif


#define PERIODIC_SEND_DOWNLINK(_chan) { \
  static uint16_t last; \
  uint16_t rate = (downlink_nb_bytes - last) / PERIOD_DOWNLINK_Ap_DefaultChannel_0; \
  last = downlink_nb_bytes; \
  DOWNLINK_SEND_DOWNLINK(_chan, &downlink_nb_ovrn, &rate, &downlink_nb_msgs); \
}


#define PERIODIC_SEND_ATTITUDE(_chan) Downlink({ \
      DOWNLINK_SEND_ATTITUDE(_chan, &estimator_phi, &estimator_psi, &estimator_theta); \
})

#define PERIODIC_SEND_PPRZ_MODE(_chan) DOWNLINK_SEND_PPRZ_MODE(_chan, &pprz_mode, &v_ctl_mode, &lateral_mode, &horizontal_mode, &rc_settings_mode, &mcu1_status);
#define PERIODIC_SEND_DESIRED(_chan) DOWNLINK_SEND_DESIRED(_chan, &h_ctl_roll_setpoint, &h_ctl_pitch_loop_setpoint, &h_ctl_course_setpoint, &desired_x, &desired_y, &v_ctl_altitude_setpoint, &v_ctl_climb_setpoint);

#define PERIODIC_SEND_NAVIGATION_REF(_chan)  DOWNLINK_SEND_NAVIGATION_REF(_chan, &nav_utm_east0, &nav_utm_north0, &nav_utm_zone0);


#define DownlinkSendWp(_chan, i) {	     \
  float x = nav_utm_east0 +  waypoints[i].x; \
  float y = nav_utm_north0 + waypoints[i].y; \
  DOWNLINK_SEND_WP_MOVED(_chan, &i, &x, &y, &(waypoints[i].a),&nav_utm_zone0); \
}


#define PERIODIC_SEND_WP_MOVED(_chan) { \
  static uint8_t i; \
  i++; if (i >= nb_waypoint) i = 0; \
  DownlinkSendWp(_chan, i);	    \
}

#ifdef RADIO_CONTROL_SETTINGS
#define PERIODIC_SEND_SETTINGS(_chan) if (!RcSettingsOff()) DOWNLINK_SEND_SETTINGS(_chan, &slider_1_val, &slider_2_val);
#else
#define PERIODIC_SEND_SETTINGS(_chan) {}
#endif

#if defined USE_INFRARED || USE_INFRARED_TELEMETRY
#define PERIODIC_SEND_IR_SENSORS(_chan) DOWNLINK_SEND_IR_SENSORS(_chan, &infrared.value.ir1, &infrared.value.ir2, &infrared.pitch, &infrared.roll, &infrared.top);
#else
#define PERIODIC_SEND_IR_SENSORS(_chan) ;
#endif

#define PERIODIC_SEND_ADC(_chan) {}

#ifdef IDG300
#include "gyro.h"
#define PERIODIC_SEND_GYRO_RATES(_chan) DOWNLINK_SEND_GYRO_RATES(_chan, &roll_rate_adc, &estimator_p, &estimator_q)
#elif defined ADXRS150
#define PERIODIC_SEND_GYRO_RATES(_chan) DOWNLINK_SEND_GYRO_RATES(_chan, &roll_rate_adc, &estimator_p, &temp_comp)
#else
#define PERIODIC_SEND_GYRO_RATES(_chan) {}
#endif

#define PERIODIC_SEND_CALIBRATION(_chan) DOWNLINK_SEND_CALIBRATION(_chan, &v_ctl_auto_throttle_sum_err, &v_ctl_auto_throttle_submode)

#define PERIODIC_SEND_CIRCLE(_chan) if (nav_in_circle) { DOWNLINK_SEND_CIRCLE(_chan, &nav_circle_x, &nav_circle_y, &nav_circle_radius); }

#define PERIODIC_SEND_SEGMENT(_chan) if (nav_in_segment) { DOWNLINK_SEND_SEGMENT(_chan, &nav_segment_x_1, &nav_segment_y_1, &nav_segment_x_2, &nav_segment_y_2); }

#ifdef IMU_TYPE_H
#include "subsystems/imu.h"
#define PERIODIC_SEND_IMU_ACCEL_RAW(_chan) { DOWNLINK_SEND_IMU_ACCEL_RAW(_chan, &imu.accel_unscaled.x, &imu.accel_unscaled.y, &imu.accel_unscaled.z)}
#define PERIODIC_SEND_IMU_GYRO_RAW(_chan) { DOWNLINK_SEND_IMU_GYRO_RAW(_chan, &imu.gyro_unscaled.p, &imu.gyro_unscaled.q, &imu.gyro_unscaled.r)}
#define PERIODIC_SEND_IMU_ACCEL(_chan) { struct FloatVect3 accel_float; ACCELS_FLOAT_OF_BFP(accel_float, imu.accel); DOWNLINK_SEND_IMU_ACCEL(_chan, &accel_float.x, &accel_float.y, &accel_float.z)}
#define PERIODIC_SEND_IMU_GYRO(_chan) { struct FloatRates gyro_float; RATES_FLOAT_OF_BFP(gyro_float, imu.gyro); DOWNLINK_SEND_IMU_GYRO(_chan, &gyro_float.p, &gyro_float.q, &gyro_float.r)}
#else
#define PERIODIC_SEND_IMU_ACCEL_RAW(_chan) {}
#define PERIODIC_SEND_IMU_GYRO_RAW(_chan) {}
#define PERIODIC_SEND_IMU_ACCEL(_chan) {}
#define PERIODIC_SEND_IMU_GYRO(_chan) {}
#endif

#ifdef IMU_ANALOG
#define PERIODIC_SEND_IMU(_chan) { int16_t dummy = 42; DOWNLINK_SEND_IMU(_chan, &(from_fbw.euler_dot[0]), &(from_fbw.euler_dot[1]), &(from_fbw.euler_dot[2]), &dummy, &dummy, &dummy); }
#else
#define PERIODIC_SEND_IMU(_chan) {}
#endif

#define PERIODIC_SEND_ESTIMATOR(_chan) DOWNLINK_SEND_ESTIMATOR(_chan, &estimator_z, &estimator_z_dot)

#define SEND_NAVIGATION(_chan) Downlink({ uint8_t _circle_count = NavCircleCount(); DOWNLINK_SEND_NAVIGATION(_chan, &nav_block, &nav_stage, &estimator_x, &estimator_y, &dist2_to_wp, &dist2_to_home, &_circle_count, &nav_oval_count);})

#define PERIODIC_SEND_NAVIGATION(_chan) SEND_NAVIGATION(_chan)

#if defined CAM || defined MOBILE_CAM
#define SEND_CAM(_chan) Downlink({ int16_t x = cam_target_x; int16_t y = cam_target_y; int8_t phi = DegOfRad(cam_phi_c); int8_t theta = DegOfRad(cam_theta_c); DOWNLINK_SEND_CAM(_chan, &phi, &theta, &x, &y);})
#else
#define SEND_CAM(_chan) {}
#endif

#define PERIODIC_SEND_DL_VALUE(_chan) PeriodicSendDlValue(_chan) /** generated from the xml settings config in conf/settings */

#define PERIODIC_SEND_SURVEY(_chan) { \
  if (nav_survey_active) \
    DOWNLINK_SEND_SURVEY(_chan, &nav_survey_east, &nav_survey_north, &nav_survey_west, &nav_survey_south); \
  }

#define PERIODIC_SEND_RANGEFINDER(_chan) DOWNLINK_SEND_RANGEFINDER(_chan, &rangemeter, &ctl_grz_z_dot, &ctl_grz_z_dot_sum_err, &ctl_grz_z_dot_setpoint, &ctl_grz_z_sum_err, &ctl_grz_z_setpoint, &flying)

#define PERIODIC_SEND_TUNE_ROLL(_chan) DOWNLINK_SEND_TUNE_ROLL(_chan, &estimator_p,&estimator_phi, &h_ctl_roll_setpoint);

#if defined USE_GPS || defined SITL || defined USE_GPS_XSENS
#define PERIODIC_SEND_GPS_SOL(_chan) DOWNLINK_SEND_GPS_SOL(_chan, &gps_Pacc, &gps_Sacc, &gps_PDOP, &gps_numSV)
#endif

/* add by Haiyang Chao for debugging msg used by osam_imu*/
#if defined UGEAR
#define PERIODIC_SEND_GPS(_chan) DOWNLINK_SEND_GPS(_chan, &gps_mode, &gps_utm_east, &gps_utm_north, &gps_course, &gps_alt, &gps_gspeed,&gps_climb, &gps_week, &gps_itow, &gps_utm_zone, &gps_nb_ovrn)
#define PERIODIC_SEND_GPS_SOL(_chan) DOWNLINK_SEND_GPS_SOL(_chan, &gps_Pacc, &gps_Sacc, &gps_PDOP, &gps_numSV)
#define PERIODIC_SEND_DebugChao(_chan) DOWNLINK_SEND_DebugChao(_chan, &ugear_debug1, &ugear_debug2, &ugear_debug3, &ugear_debug4, &ugear_debug5, &ugear_debug6)
#else
#define PERIODIC_SEND_GPS(_chan) gps_send()
#endif

#ifdef USE_BARO_MS5534A
//#include "baro_MS5534A.h"
#define PERIODIC_SEND_BARO_MS5534A(_chan) DOWNLINK_SEND_BARO_MS5534A(_chan, &baro_MS5534A_pressure, &baro_MS5534A_temp, &baro_MS5534A_z)
#else
#define PERIODIC_SEND_BARO_MS5534A(_chan) {}
#endif

#ifdef USE_BARO_SCP
#include "baro_scp.h"
#define PERIODIC_SEND_SCP_STATUS(_chan) DOWNLINK_SEND_SCP_STATUS(_chan, &baro_scp_pressure, &baro_scp_temperature)
#else
#define PERIODIC_SEND_SCP_STATUS(_chan) {}
#endif

#ifdef MEASURE_AIRSPEED
#define PERIODIC_SEND_AIRSPEED(_chan) DOWNLINK_SEND_AIRSPEED (_chan, &estimator_airspeed, &estimator_airspeed, &estimator_airspeed, &estimator_airspeed)
#elif defined USE_AIRSPEED
#define PERIODIC_SEND_AIRSPEED(_chan) DOWNLINK_SEND_AIRSPEED (_chan, &estimator_airspeed, &v_ctl_auto_airspeed_setpoint, &v_ctl_auto_airspeed_controlled, &v_ctl_auto_groundspeed_setpoint)
#else
#define PERIODIC_SEND_AIRSPEED(_chan) {}
#endif

#define PERIODIC_SEND_ENERGY(_chan) Downlink({ const int16_t e = energy; const float vsup = ((float)vsupply) / 10.0f; const float curs = ((float) current)/1000.0f;  const float power = vsup * curs; DOWNLINK_SEND_ENERGY(_chan, &vsup, &curs, &e, &power); })


#include "firmwares/fixedwing/stabilization/stabilization_adaptive.h"
#define PERIODIC_SEND_H_CTL_A(_chan) DOWNLINK_SEND_H_CTL_A(_chan, &h_ctl_roll_sum_err, &h_ctl_ref_roll_angle, &h_ctl_pitch_sum_err, &h_ctl_ref_pitch_angle)


#endif /* AP_DOWNLINK_H */
