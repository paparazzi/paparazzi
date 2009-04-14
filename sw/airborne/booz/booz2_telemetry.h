/*
 * $Id$
 *
 * Copyright (C) 2008  Antoine Drouin
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

#ifndef BOOZ2_TELEMETRY_H
#define BOOZ2_TELEMETRY_H

#include "std.h"
#include "messages.h"
#include "uart.h"

#include "downlink.h"

#include "radio_control.h"
#include "booz2_autopilot.h"
#include "booz2_guidance_h.h"
#include "booz2_guidance_v.h"

#include "actuators.h"

#define PERIODIC_SEND_ALIVE() DOWNLINK_SEND_ALIVE(16, MD5SUM)

#include "booz2_battery.h"
#include "booz2_imu.h"
#include "booz2_gps.h"
#include "booz2_ins.h"
#define PERIODIC_SEND_BOOZ_STATUS() {					\
    uint32_t booz_imu_nb_err = 0;					\
    DOWNLINK_SEND_BOOZ_STATUS(&booz_imu_nb_err,				\
			      &twi_blmc_nb_err,				\
			      &rc_status,				\
			      &booz_gps_state.fix,			\
			      &booz2_autopilot_mode,			\
			      &booz2_autopilot_in_flight,		\
			      &booz2_autopilot_motors_on,		\
			      &booz2_guidance_h_mode,			\
			      &booz2_guidance_v_mode,			\
			      &booz2_battery_voltage,			\
			      &cpu_time_sec				\
			      );					\
  }

#ifdef RADIO_CONTROL
#define PERIODIC_SEND_PPM() DOWNLINK_SEND_PPM(&last_ppm_cpt, PPM_NB_PULSES, ppm_pulses)
#define PERIODIC_SEND_RC() DOWNLINK_SEND_RC(PPM_NB_PULSES, rc_values)
#else // RADIO_CONTROL
#define PERIODIC_SEND_PPM() {}
#define PERIODIC_SEND_RC() {}
#endif // RADIO_CONTROL

#define PERIODIC_SEND_BOOZ2_GYRO() {			\
    DOWNLINK_SEND_BOOZ2_GYRO(&booz_imu.gyro.p,		\
			     &booz_imu.gyro.q,		\
			     &booz_imu.gyro.r);		\
  }

#define PERIODIC_SEND_BOOZ2_ACCEL() {				\
    DOWNLINK_SEND_BOOZ2_ACCEL(&booz_imu.accel.x,		\
			      &booz_imu.accel.y,		\
			      &booz_imu.accel.z);		\
  }

#define PERIODIC_SEND_BOOZ2_MAG() {				\
    DOWNLINK_SEND_BOOZ2_MAG(&booz_imu.mag.x,			\
			    &booz_imu.mag.y,			\
			    &booz_imu.mag.z);			\
  }



#define PERIODIC_SEND_IMU_GYRO_RAW() {					\
    DOWNLINK_SEND_IMU_GYRO_RAW(&booz_imu.gyro_unscaled.p,		\
			       &booz_imu.gyro_unscaled.q,		\
			       &booz_imu.gyro_unscaled.r);		\
  }

#define PERIODIC_SEND_IMU_ACCEL_RAW() {					\
    DOWNLINK_SEND_IMU_ACCEL_RAW(&booz_imu.accel_unscaled.x,		\
				&booz_imu.accel_unscaled.y,		\
				&booz_imu.accel_unscaled.z);		\
  }

#define PERIODIC_SEND_IMU_MAG_RAW() {					\
    DOWNLINK_SEND_IMU_MAG_RAW(&booz_imu.mag_unscaled.x,			\
			      &booz_imu.mag_unscaled.y,			\
			      &booz_imu.mag_unscaled.z);		\
  }




#include "booz2_imu.h"
#include "booz2_stabilization.h"
#include "booz2_stabilization_rate.h"
#define PERIODIC_SEND_BOOZ2_RATE_LOOP() {				\
    DOWNLINK_SEND_BOOZ2_RATE_LOOP(&booz2_stabilization_rate_measure.p,	\
				  &booz2_stabilization_rate_measure.q,	\
				  &booz2_stabilization_rate_measure.r,	\
				  &booz2_stabilization_rate_sp.p,	\
				  &booz2_stabilization_rate_sp.q,	\
				  &booz2_stabilization_rate_sp.r,	\
				  &booz2_stabilization_cmd[COMMAND_ROLL], \
				  &booz2_stabilization_cmd[COMMAND_PITCH], \
				  &booz2_stabilization_cmd[COMMAND_YAW], \
				  &booz2_stabilization_cmd[COMMAND_THRUST]); \
  }


#include "booz2_stabilization_attitude.h"
#define PERIODIC_SEND_BOOZ2_STAB_ATTITUDE() {				\
    DOWNLINK_SEND_BOOZ2_STAB_ATTITUDE(&booz_ahrs.body_rate.p,		\
				      &booz_ahrs.body_rate.q,		\
				      &booz_ahrs.body_rate.r,		\
				      &booz_ahrs.ltp_to_body_euler.phi,	\
				      &booz_ahrs.ltp_to_body_euler.theta, \
				      &booz_ahrs.ltp_to_body_euler.psi,	\
                                      &booz_stabilization_att_sp.phi,	\
				      &booz_stabilization_att_sp.theta,	\
				      &booz_stabilization_att_sp.psi,	\
				      &booz_stabilization_att_sum_err.phi, \
				      &booz_stabilization_att_sum_err.theta, \
				      &booz_stabilization_att_sum_err.psi, \
				      &booz2_stabilization_cmd[COMMAND_ROLL], \
				      &booz2_stabilization_cmd[COMMAND_PITCH], \
				      &booz2_stabilization_cmd[COMMAND_YAW], \
				      &booz2_stabilization_cmd[COMMAND_THRUST]); \
  }


#define PERIODIC_SEND_BOOZ2_STAB_ATTITUDE_REF() {			     \
    DOWNLINK_SEND_BOOZ2_STAB_ATTITUDE_REF(&booz_stabilization_accel_ref.x,   \
					  &booz_stabilization_accel_ref.y,   \
					  &booz_stabilization_accel_ref.z,   \
					  &booz_stabilization_rate_ref.x,    \
					  &booz_stabilization_rate_ref.y,    \
					  &booz_stabilization_rate_ref.z,    \
					  &booz_stabilization_att_ref.phi,   \
					  &booz_stabilization_att_ref.theta, \
					  &booz_stabilization_att_ref.psi ); \
  }

#if defined HS_YAW
#define PERIODIC_SEND_BOOZ2_STAB_ATTITUDE_HS_ROLL() {			\
    DOWNLINK_SEND_BOOZ2_STAB_ATTITUDE_HS_ROLL(&booz_stabilization_att_ref.psi, \
					      &booz_stabilization_rate_ref.z, \
					      &booz_stabilization_accel_ref.z, \
					      &booz_ahrs.ltp_to_body_euler.psi,	\
					      &booz_ahrs.body_rate.r, \
					      &booz_stabilization_att_sum_err.psi, \
					      &booz2_stabilization_cmd[COMMAND_YAW]); \
  }
#else
#define PERIODIC_SEND_BOOZ2_STAB_ATTITUDE_HS_ROLL() {			       \
    DOWNLINK_SEND_BOOZ2_STAB_ATTITUDE_HS_ROLL(&booz_stabilization_att_ref.phi, \
					      &booz_stabilization_rate_ref.x,  \
					      &booz_stabilization_accel_ref.x, \
					      &booz_ahrs.ltp_to_body_euler.phi,	\
					      &booz_ahrs.body_rate.p, \
					      &booz_stabilization_att_sum_err.phi, \
					      &booz2_stabilization_cmd[COMMAND_ROLL]); \
  }
#endif


#include "booz_ahrs_aligner.h"
#define PERIODIC_SEND_BOOZ2_FILTER_ALIGNER() {				\
    DOWNLINK_SEND_BOOZ2_FILTER_ALIGNER(&booz_ahrs_aligner.lp_gyro.p,	\
				       &booz_ahrs_aligner.lp_gyro.q,	\
				       &booz_ahrs_aligner.lp_gyro.r,	\
				       &booz_imu.gyro.p,		\
				       &booz_imu.gyro.q,		\
				       &booz_imu.gyro.r,		\
				       &booz_ahrs_aligner.noise,	\
				       &booz_ahrs_aligner.low_noise_cnt); \
  }


#define PERIODIC_SEND_BOOZ2_CMD() { \
    DOWNLINK_SEND_BOOZ2_CMD(&booz2_stabilization_cmd[COMMAND_ROLL],	\
			    &booz2_stabilization_cmd[COMMAND_PITCH],	\
			    &booz2_stabilization_cmd[COMMAND_YAW],	\
			    &booz2_stabilization_cmd[COMMAND_THRUST]);	\
  }



#include "booz_ahrs.h"
#include "booz2_filter_attitude_cmpl_euler.h"
#define PERIODIC_SEND_BOOZ2_FILTER() {					\
    DOWNLINK_SEND_BOOZ2_FILTER(&booz_ahrs.ltp_to_imu_euler.phi,	\
			       &booz_ahrs.ltp_to_imu_euler.theta,	\
			       &booz_ahrs.ltp_to_imu_euler.psi,	\
			       &booz2_face_measure.phi,			\
			       &booz2_face_measure.theta,		\
			       &booz2_face_measure.psi,			\
			       &booz2_face_corrected.phi,		\
			       &booz2_face_corrected.theta,		\
			       &booz2_face_corrected.psi,		\
			       &booz2_face_residual.phi,		\
			       &booz2_face_residual.theta,		\
			       &booz2_face_residual.psi,		\
			       &booz2_face_gyro_bias.p,			\
			       &booz2_face_gyro_bias.q,			\
			       &booz2_face_gyro_bias.r);		\
  }


#define PERIODIC_SEND_BOOZ2_AHRS_QUAT() {				\
    DOWNLINK_SEND_BOOZ2_AHRS_QUAT(&booz_ahrs.ltp_to_imu_quat.qi,	\
				  &booz_ahrs.ltp_to_imu_quat.qx,	\
				  &booz_ahrs.ltp_to_imu_quat.qy,	\
				  &booz_ahrs.ltp_to_imu_quat.qz,	\
				  &booz_ahrs.ltp_to_body_quat.qi,	\
				  &booz_ahrs.ltp_to_body_quat.qx,	\
				  &booz_ahrs.ltp_to_body_quat.qy,	\
				  &booz_ahrs.ltp_to_body_quat.qz);	\
  }

#define PERIODIC_SEND_BOOZ2_AHRS_EULER() {				\
    DOWNLINK_SEND_BOOZ2_AHRS_EULER(&booz_ahrs.ltp_to_imu_euler.phi,	\
				   &booz_ahrs.ltp_to_imu_euler.theta,	\
				   &booz_ahrs.ltp_to_imu_euler.psi,	\
				   &booz_ahrs.ltp_to_body_euler.phi,	\
				   &booz_ahrs.ltp_to_body_euler.theta,	\
				   &booz_ahrs.ltp_to_body_euler.psi);	\
  }

#define PERIODIC_SEND_BOOZ2_AHRS_RMAT() {				\
    DOWNLINK_SEND_BOOZ2_AHRS_RMAT(&booz_ahrs.ltp_to_imu_rmat.m[0],	\
				  &booz_ahrs.ltp_to_imu_rmat.m[1],	\
				  &booz_ahrs.ltp_to_imu_rmat.m[2],	\
				  &booz_ahrs.ltp_to_imu_rmat.m[3],	\
				  &booz_ahrs.ltp_to_imu_rmat.m[4],	\
				  &booz_ahrs.ltp_to_imu_rmat.m[5],	\
				  &booz_ahrs.ltp_to_imu_rmat.m[6],	\
				  &booz_ahrs.ltp_to_imu_rmat.m[7],	\
				  &booz_ahrs.ltp_to_imu_rmat.m[8],	\
				  &booz_ahrs.ltp_to_body_rmat.m[0],	\
				  &booz_ahrs.ltp_to_body_rmat.m[1],	\
				  &booz_ahrs.ltp_to_body_rmat.m[2],	\
				  &booz_ahrs.ltp_to_body_rmat.m[3],	\
				  &booz_ahrs.ltp_to_body_rmat.m[4],	\
				  &booz_ahrs.ltp_to_body_rmat.m[5],	\
				  &booz_ahrs.ltp_to_body_rmat.m[6],	\
				  &booz_ahrs.ltp_to_body_rmat.m[7],	\
				  &booz_ahrs.ltp_to_body_rmat.m[8]);	\
  }




#define PERIODIC_SEND_BOOZ2_FILTER_Q() {				\
    DOWNLINK_SEND_BOOZ2_FILTER_Q(&booz2_filter_attitude_quat.qi,	\
				 &booz2_filter_attitude_quat.qx,	\
				 &booz2_filter_attitude_quat.qy,	\
				 &booz2_filter_attitude_quat.qz);	\
  }

#include "booz2_guidance_h.h"
#define PERIODIC_SEND_BOOZ2_GUIDANCE() {				\
    DOWNLINK_SEND_BOOZ2_GUIDANCE(&booz2_guidance_h_cur_pos.x,		\
				 &booz2_guidance_h_cur_pos.y,		\
				 &booz2_guidance_h_held_pos.x,		\
				 &booz2_guidance_h_held_pos.y);		\
  }

#include "booz2_ins.h"
#define PERIODIC_SEND_BOOZ2_INS() {				\
    DOWNLINK_SEND_BOOZ2_INS(&booz_ins_baro_alt,			\
			    &booz_ins_ltp_pos.z,		\
			    &booz_ins_ltp_speed.z,		\
			    &booz_ins_ltp_accel.z);		\
  }


#include "booz2_hf_float.h"
#define PERIODIC_SEND_BOOZ2_INS2() {				\
    struct Int32Vect3 pos_low_res;				\
    pos_low_res.x = (int32_t)(b2ins_pos_ltp.x>>20);		\
    pos_low_res.y = (int32_t)(b2ins_pos_ltp.y>>20);		\
    pos_low_res.z = (int32_t)(b2ins_pos_ltp.z>>20);		\
    DOWNLINK_SEND_BOOZ2_INS2(&b2ins_accel_ltp.x,		\
			     &b2ins_accel_ltp.y,		\
			     &b2ins_accel_ltp.z,		\
			     &b2ins_speed_ltp.x,		\
			     &b2ins_speed_ltp.y,		\
			     &b2ins_speed_ltp.z,		\
			     &pos_low_res.x,			\
			     &pos_low_res.y,			\
			     &pos_low_res.z			\
			     );					\
  }

#define PERIODIC_SEND_BOOZ2_INS3() {					\
    DOWNLINK_SEND_BOOZ2_INS3(&b2ins_meas_gps_pos_ned.x,			\
			     &b2ins_meas_gps_pos_ned.y,			\
			     &b2ins_meas_gps_pos_ned.z,			\
			     &b2ins_meas_gps_speed_ned.x,		\
			     &b2ins_meas_gps_speed_ned.y,		\
			     &b2ins_meas_gps_speed_ned.z		\
			     );						\
  }


#define PERIODIC_SEND_BOOZ2_INS_REF() {					\
    DOWNLINK_SEND_BOOZ2_INS_REF(&booz_ins_ltp_def.ecef.x,		\
				&booz_ins_ltp_def.ecef.y,		\
				&booz_ins_ltp_def.ecef.z,		\
				&booz_ins_ltp_def.lla.lat,		\
				&booz_ins_ltp_def.lla.lon,		\
				&booz_ins_ltp_def.lla.alt,		\
				&booz_ins_qfe);				\
  }



#include "booz2_guidance_v.h"
#define PERIODIC_SEND_BOOZ2_VERT_LOOP() {				\
    DOWNLINK_SEND_BOOZ2_VERT_LOOP(&booz2_guidance_v_z_sp,		\
				  &booz2_guidance_v_zd_sp,		\
				  &booz_ins_ltp_pos.z,			\
				  &booz_ins_ltp_speed.z,		\
				  &booz_ins_ltp_accel.z,		\
				  &booz2_guidance_v_z_ref,		\
				  &booz2_guidance_v_zd_ref,		\
				  &booz2_guidance_v_zdd_ref,		\
				  &b2_gv_adapt_X,			\
				  &b2_gv_adapt_P,			\
				  &b2_gv_adapt_Xmeas,			\
				  &booz2_guidance_v_z_sum_err,		\
				  &booz2_guidance_v_ff_cmd,		\
				  &booz2_guidance_v_fb_cmd,		\
				  &booz2_guidance_v_delta_t);		\
  }

#define PERIODIC_SEND_BOOZ2_HOVER_LOOP() {				\
    DOWNLINK_SEND_BOOZ2_HOVER_LOOP(&booz2_guidance_h_pos_sp.x,		\
				   &booz2_guidance_h_pos_sp.y,		\
				   &booz_ins_ltp_pos.x,			\
				   &booz_ins_ltp_pos.y,			\
				   &booz_ins_ltp_speed.x,		\
				   &booz_ins_ltp_speed.y,		\
				   &booz2_guidance_h_pos_err.x,		\
				   &booz2_guidance_h_pos_err.y,		\
				   &booz2_guidance_h_speed_err.x,		\
				   &booz2_guidance_h_speed_err.y,		\
				   &booz2_guidance_h_pos_err_sum.x,	\
				   &booz2_guidance_h_pos_err_sum.y,	\
				   &booz2_guidance_h_command_earth.x,	\
				   &booz2_guidance_h_command_earth.y,	\
				   &booz2_guidance_h_command_body.phi,	\
				   &booz2_guidance_h_command_body.theta, \
				   &booz2_guidance_h_command_body.psi);	\
  }


#include "booz2_gps.h"
#include "booz2_navigation.h"
#define PERIODIC_SEND_BOOZ2_FP() {					\
    int32_t carrot_up = -booz2_guidance_v_z_sp; \
    DOWNLINK_SEND_BOOZ2_FP( &booz_ins_enu_pos.x,			\
			    &booz_ins_enu_pos.y,			\
			    &booz_ins_enu_pos.z,			\
			    &booz_ins_enu_speed.x,			\
			    &booz_ins_enu_speed.y,			\
			    &booz_ins_enu_speed.z,			\
			    &booz_ahrs.ltp_to_body_euler.phi,		\
			    &booz_ahrs.ltp_to_body_euler.theta,		\
			    &booz_ahrs.ltp_to_body_euler.psi,		\
			    &booz2_guidance_h_pos_sp.y,			\
			    &booz2_guidance_h_pos_sp.x,			\
			    &carrot_up,			\
			    &booz2_guidance_h_command_body.psi,		\
			    &booz2_stabilization_cmd[COMMAND_THRUST]);	\
  }


#define PERIODIC_SEND_BOOZ2_GPS() {				\
    DOWNLINK_SEND_BOOZ2_GPS( &booz_gps_state.ecef_pos.x,	\
			     &booz_gps_state.ecef_pos.y,	\
			     &booz_gps_state.ecef_pos.z,	\
			     &booz_gps_state.ecef_speed.x,	\
			     &booz_gps_state.ecef_speed.y,	\
			     &booz_gps_state.ecef_speed.z,	\
			     &booz_gps_state.pacc,		\
			     &booz_gps_state.sacc,		\
			     &booz_gps_state.pdop,		\
			     &booz_gps_state.num_sv,		\
			     &booz_gps_state.fix);		\
  }


#include "booz2_navigation.h"
#define PERIODIC_SEND_BOOZ2_NAV_REF() {					\
    DOWNLINK_SEND_BOOZ2_NAV_REF(&booz_ins_ltp_def.ecef.x, &booz_ins_ltp_def.ecef.y, &booz_ins_ltp_def.ecef.z);	\
  }

#define PERIODIC_SEND_BOOZ2_NAV_STATUS() {					\
    DOWNLINK_SEND_BOOZ2_NAV_STATUS(&block_time,&stage_time,&nav_block,&nav_stage,&horizontal_mode); \
    if (horizontal_mode == HORIZONTAL_MODE_ROUTE) { \
      int32_t sx = waypoints[nav_segment_start].x >> INT32_POS_FRAC; \
      int32_t sy = waypoints[nav_segment_start].y >> INT32_POS_FRAC; \
      int32_t ex = waypoints[nav_segment_end].x >> INT32_POS_FRAC; \
      int32_t ey = waypoints[nav_segment_end].y >> INT32_POS_FRAC; \
      DOWNLINK_SEND_SEGMENT(&sx, &sy, &ex, &ey); \
    } \
  }

#define PERIODIC_SEND_WP_MOVED() { \
  static uint8_t i; \
  i++; if (i >= nb_waypoint) i = 0; \
  DOWNLINK_SEND_WP_MOVED_LTP(&i, &(waypoints[i].x), &(waypoints[i].y), &(waypoints[i].z)); \
}


#define PERIODIC_SEND_BOOZ2_TUNE_HOVER() {				\
    DOWNLINK_SEND_BOOZ2_TUNE_HOVER(&rc_values[RADIO_ROLL],		\
				   &rc_values[RADIO_PITCH],		\
				   &rc_values[RADIO_YAW],		\
				   &booz2_stabilization_cmd[COMMAND_ROLL], \
				   &booz2_stabilization_cmd[COMMAND_PITCH], \
				   &booz2_stabilization_cmd[COMMAND_YAW], \
				   &booz2_stabilization_cmd[COMMAND_THRUST], \
				   &booz_ahrs.ltp_to_imu_euler.phi,	\
				   &booz_ahrs.ltp_to_imu_euler.theta,	\
				   &booz_ahrs.ltp_to_imu_euler.psi,	\
				   &booz_ahrs.ltp_to_body_euler.phi,	\
				   &booz_ahrs.ltp_to_body_euler.theta,	\
				   &booz_ahrs.ltp_to_body_euler.psi	\
				   );					\
  }




#include "settings.h"
#define PERIODIC_SEND_DL_VALUE() PeriodicSendDlValue()

extern uint8_t telemetry_mode_Main;

#include "periodic.h"
#define Booz2TelemetryPeriodic() {		\
    PeriodicSendMain();				\
  }


#endif /* BOOZ2_TELEMETRY_H */
