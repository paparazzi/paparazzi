/*
 * $Id$
 *
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

#ifndef BOOZ2_TELEMETRY_H
#define BOOZ2_TELEMETRY_H

#include "std.h"
#include "messages.h"
#include "uart.h"

#include "downlink.h"

#include "booz_radio_control.h"
#include "booz2_autopilot.h"
#include "booz_guidance.h"

#include "actuators.h"

#define PERIODIC_SEND_ALIVE() DOWNLINK_SEND_ALIVE(16, MD5SUM)

#include "booz2_battery.h"
#include "booz_imu.h"
#include "booz2_gps.h"
#include "booz2_ins.h"

#ifdef USE_GPS
#define PERIODIC_SEND_BOOZ_STATUS() {					\
    uint32_t booz_imu_nb_err = 0;					\
    DOWNLINK_SEND_BOOZ_STATUS(&booz_imu_nb_err,				\
			      &twi_blmc_nb_err,				\
			      &radio_control.status,			\
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
#else /* !USE_GPS */
#define PERIODIC_SEND_BOOZ_STATUS() {					\
    uint32_t booz_imu_nb_err = 0;					\
    uint8_t  fix = BOOZ2_GPS_FIX_NONE;					\
    DOWNLINK_SEND_BOOZ_STATUS(&booz_imu_nb_err,				\
			      &twi_blmc_nb_err,				\
			      &radio_control.status,			\
			      &fix,					\
			      &booz2_autopilot_mode,			\
			      &booz2_autopilot_in_flight,		\
			      &booz2_autopilot_motors_on,		\
			      &booz2_guidance_h_mode,			\
			      &booz2_guidance_v_mode,			\
			      &booz2_battery_voltage,			\
			      &cpu_time_sec				\
			      );					\
  }
#endif /* USE_GPS */

#ifdef USE_RADIO_CONTROL
#define PERIODIC_SEND_RC() DOWNLINK_SEND_RC(RADIO_CONTROL_NB_CHANNEL, radio_control.values)
#define PERIODIC_SEND_BOOZ2_RADIO_CONTROL() {				\
    DOWNLINK_SEND_BOOZ2_RADIO_CONTROL(&radio_control.values[RADIO_CONTROL_ROLL], \
				      &radio_control.values[RADIO_CONTROL_PITCH], \
				      &radio_control.values[RADIO_CONTROL_YAW], \
				      &radio_control.values[RADIO_CONTROL_THROTTLE], \
				      &radio_control.values[RADIO_CONTROL_MODE], \
				      &radio_control.status);}

#else
#define PERIODIC_SEND_RC() {}
#endif

#ifdef RADIO_CONTROL_TYPE_PPM
#define PERIODIC_SEND_PPM() DOWNLINK_SEND_PPM(&radio_control.frame_rate, \
					      RADIO_CONTROL_NB_CHANNEL,  \
					      booz_radio_control_ppm_pulses)
#else
#define PERIODIC_SEND_PPM() {}
#endif

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

#define PERIODIC_SEND_BOOZ2_BARO_RAW() {					\
    DOWNLINK_SEND_BOOZ2_BARO_RAW(&booz2_analog_baro_offset,			\
			         &booz2_analog_baro_value,			\
			         &booz2_analog_baro_value_filtered);		\
  }

#include "booz_stabilization.h"
#define PERIODIC_SEND_BOOZ2_RATE_LOOP() {				\
    DOWNLINK_SEND_BOOZ2_RATE_LOOP(&booz_stabilization_rate_measure.p,	\
				  &booz_stabilization_rate_measure.q,	\
				  &booz_stabilization_rate_measure.r,	\
				  &booz_stabilization_rate_sp.p,	\
				  &booz_stabilization_rate_sp.q,	\
				  &booz_stabilization_rate_sp.r,	\
				  &booz_stabilization_cmd[COMMAND_ROLL], \
				  &booz_stabilization_cmd[COMMAND_PITCH], \
				  &booz_stabilization_cmd[COMMAND_YAW], \
				  &booz_stabilization_cmd[COMMAND_THRUST]); \
  }

#ifdef STABILISATION_ATTITUDE_TYPE_INT
#define PERIODIC_SEND_BOOZ2_STAB_ATTITUDE() {				\
    DOWNLINK_SEND_BOOZ2_STAB_ATTITUDE_INT(&booz_ahrs.body_rate.p,	\
					  &booz_ahrs.body_rate.q,	\
					  &booz_ahrs.body_rate.r,	\
					  &booz_ahrs.ltp_to_body_euler.phi, \
					  &booz_ahrs.ltp_to_body_euler.theta, \
					  &booz_ahrs.ltp_to_body_euler.psi, \
					  &booz_stab_att_sp_euler.phi, \
					  &booz_stab_att_sp_euler.theta, \
					  &booz_stab_att_sp_euler.psi, \
					  &booz_stabilization_att_sum_err.phi, \
					  &booz_stabilization_att_sum_err.theta, \
					  &booz_stabilization_att_sum_err.psi, \
					  &booz_stabilization_att_fb_cmd[COMMAND_ROLL], \
					  &booz_stabilization_att_fb_cmd[COMMAND_PITCH], \
					  &booz_stabilization_att_fb_cmd[COMMAND_YAW], \
					  &booz_stabilization_att_ff_cmd[COMMAND_ROLL], \
					  &booz_stabilization_att_ff_cmd[COMMAND_PITCH], \
					  &booz_stabilization_att_ff_cmd[COMMAND_YAW], \
					  &booz_stabilization_cmd[COMMAND_ROLL], \
					  &booz_stabilization_cmd[COMMAND_PITCH], \
					  &booz_stabilization_cmd[COMMAND_YAW]); \
  }


#define PERIODIC_SEND_BOOZ2_STAB_ATTITUDE_REF() {			\
    DOWNLINK_SEND_BOOZ2_STAB_ATTITUDE_REF_INT(&booz_stab_att_sp_euler.phi, \
					      &booz_stab_att_sp_euler.theta, \
					      &booz_stab_att_sp_euler.psi, \
					      &booz_stab_att_ref_euler.phi, \
					      &booz_stab_att_ref_euler.theta, \
					      &booz_stab_att_ref_euler.psi, \
					      &booz_stab_att_ref_rate.p, \
					      &booz_stab_att_ref_rate.q, \
					      &booz_stab_att_ref_rate.r, \
					      &booz_stab_att_ref_accel.p, \
					      &booz_stab_att_ref_accel.q, \
					      &booz_stab_att_ref_accel.r); \
  }
#endif /* STABILISATION_ATTITUDE_TYPE_INT */

#ifdef STABILISATION_ATTITUDE_TYPE_FLOAT
#define PERIODIC_SEND_BOOZ2_STAB_ATTITUDE() {				\
    DOWNLINK_SEND_BOOZ2_STAB_ATTITUDE_FLOAT(&booz_ahrs.body_rate.p,	\
					    &booz_ahrs.body_rate.q,	\
					    &booz_ahrs.body_rate.r,	\
					    &booz_ahrs.ltp_to_body_euler.phi, \
					    &booz_ahrs.ltp_to_body_euler.theta, \
					    &booz_ahrs.ltp_to_body_euler.psi, \
					    &booz_stab_att_ref_euler.phi, \
					    &booz_stab_att_ref_euler.theta, \
					    &booz_stab_att_ref_euler.psi, \
					    &booz_stabilization_att_sum_err.phi, \
					    &booz_stabilization_att_sum_err.theta, \
					    &booz_stabilization_att_sum_err.psi, \
					    &booz_stabilization_att_fb_cmd[COMMAND_ROLL], \
					    &booz_stabilization_att_fb_cmd[COMMAND_PITCH], \
					    &booz_stabilization_att_fb_cmd[COMMAND_YAW], \
					    &booz_stabilization_att_ff_cmd[COMMAND_ROLL], \
					    &booz_stabilization_att_ff_cmd[COMMAND_PITCH], \
					    &booz_stabilization_att_ff_cmd[COMMAND_YAW], \
					    &booz_stabilization_cmd[COMMAND_ROLL], \
					    &booz_stabilization_cmd[COMMAND_PITCH], \
					    &booz_stabilization_cmd[COMMAND_YAW]); \
  }

#define PERIODIC_SEND_BOOZ2_STAB_ATTITUDE_REF() {			\
    DOWNLINK_SEND_BOOZ2_STAB_ATTITUDE_REF_FLOAT(&booz_stab_att_sp_euler.phi, \
						&booz_stab_att_sp_euler.theta, \
						&booz_stab_att_sp_euler.psi, \
						&booz_stab_att_ref_euler.phi, \
						&booz_stab_att_ref_euler.theta, \
						&booz_stab_att_ref_euler.psi, \
						&booz_stab_att_ref_rate.p,	\
						&booz_stab_att_ref_rate.q,	\
						&booz_stab_att_ref_rate.r,	\
						&booz_stab_att_ref_accel.p, \
						&booz_stab_att_ref_accel.q, \
						&booz_stab_att_ref_accel.r); \
  }

#endif /* STABILISATION_ATTITUDE_TYPE_FLOAT */


#include "ahrs/booz_ahrs_aligner.h"
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
    DOWNLINK_SEND_BOOZ2_CMD(&booz_stabilization_cmd[COMMAND_ROLL],	\
			    &booz_stabilization_cmd[COMMAND_PITCH],	\
			    &booz_stabilization_cmd[COMMAND_YAW],	\
			    &booz_stabilization_cmd[COMMAND_THRUST]);	\
  }


#ifdef USE_AHRS_CMPL
#include "booz_ahrs.h"
#include "ahrs/booz2_filter_attitude_cmpl_euler.h"
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
#else
#define PERIODIC_SEND_BOOZ2_FILTER() {}
#endif

#ifdef USE_AHRS_LKF
#include "booz_ahrs.h"
#include "ahrs/booz_ahrs_float_lkf.h"
#define PERIODIC_SEND_BOOZ_AHRS_LKF() {					\
    DOWNLINK_SEND_BOOZ_AHRS_LKF(&bafl_eulers.phi,	\
                   &bafl_eulers.theta,  \
                   &bafl_eulers.psi,    \
                   &bafl_quat.qi,       \
                   &bafl_quat.qx,       \
                   &bafl_quat.qy,       \
                   &bafl_quat.qz,       \
                   &bafl_rates.p,       \
                   &bafl_rates.q,       \
                   &bafl_rates.r,       \
                   &bafl_accel.x,       \
                   &bafl_accel.y,       \
                   &bafl_accel.z,       \
                   &bafl_mag.x,         \
                   &bafl_mag.y,         \
                   &bafl_mag.z);         \
  }
#define PERIODIC_SEND_BOOZ_AHRS_LKF_DEBUG() {                 \
    DOWNLINK_SEND_BOOZ_AHRS_LKF_DEBUG(&bafl_X[0],          \
                   &bafl_X[1],          \
                   &bafl_X[2],          \
                   &bafl_bias.p,        \
                   &bafl_bias.q,        \
                   &bafl_bias.r,       \
                   &bafl_bias_err.p,    \
                   &bafl_bias_err.q,    \
                   &bafl_bias_err.r,    \
                   &bafl_qnorm,			\
                   &bafl_P[0][0],       \
                   &bafl_P[1][1],       \
                   &bafl_P[2][2],       \
                   &bafl_P[3][3],       \
                   &bafl_P[4][4],       \
                   &bafl_P[5][5]);      \
  }
#else
#define PERIODIC_SEND_BOOZ_AHRS_LKF() {}
#define PERIODIC_SEND_BOOZ_AHRS_LKF_DEBUG() {}
#endif


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

#ifdef USE_VFF
#include "ins/booz2_vf_float.h"
#define PERIODIC_SEND_BOOZ2_VFF() {		\
    DOWNLINK_SEND_BOOZ2_VFF(&b2_vff_z_meas,	\
			    &b2_vff_z,		\
			    &b2_vff_zdot,	\
			    &b2_vff_bias,	\
			    & b2_vff_P[0][0],	\
			    & b2_vff_P[1][1],	\
			    & b2_vff_P[2][2]);	\
  }
#else
define PERIODIC_SEND_BOOZ2_VFF() {}
#endif


#define PERIODIC_SEND_BOOZ2_GUIDANCE() {				\
    DOWNLINK_SEND_BOOZ2_GUIDANCE(&booz2_guidance_h_cur_pos.x,		\
				 &booz2_guidance_h_cur_pos.y,		\
				 &booz2_guidance_h_held_pos.x,		\
				 &booz2_guidance_h_held_pos.y);		\
  }

#define PERIODIC_SEND_BOOZ2_INS() {				\
    DOWNLINK_SEND_BOOZ2_INS(&booz_ins_baro_alt,			\
			    &booz_ins_ltp_pos.z,		\
			    &booz_ins_ltp_speed.z,		\
			    &booz_ins_ltp_accel.z);		\
  }


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

#ifdef USE_GPS
#define PERIODIC_SEND_BOOZ2_INS3() {					\
    DOWNLINK_SEND_BOOZ2_INS3(&b2ins_meas_gps_pos_ned.x,			\
			     &b2ins_meas_gps_pos_ned.y,			\
			     &b2ins_meas_gps_pos_ned.z,			\
			     &b2ins_meas_gps_speed_ned.x,		\
			     &b2ins_meas_gps_speed_ned.y,		\
			     &b2ins_meas_gps_speed_ned.z		\
			     );						\
  }
#else /* !USE_GPS */
#define PERIODIC_SEND_BOOZ2_INS3() {}
#endif /* USE_GPS */

#define PERIODIC_SEND_BOOZ2_INS_REF() {					\
    DOWNLINK_SEND_BOOZ2_INS_REF(&booz_ins_ltp_def.ecef.x,		\
				&booz_ins_ltp_def.ecef.y,		\
				&booz_ins_ltp_def.ecef.z,		\
				&booz_ins_ltp_def.lla.lat,		\
				&booz_ins_ltp_def.lla.lon,		\
				&booz_ins_ltp_def.lla.alt,		\
				&booz_ins_qfe);				\
  }



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
				   &booz2_guidance_h_speed_err.x,	\
				   &booz2_guidance_h_speed_err.y,	\
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
    int32_t carrot_up = -booz2_guidance_v_z_sp;				\
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
			    &carrot_up,					\
			    &booz2_guidance_h_command_body.psi,		\
			    &booz_stabilization_cmd[COMMAND_THRUST]);	\
  }

#ifdef USE_GPS
#define PERIODIC_SEND_BOOZ2_GPS() {				\
    DOWNLINK_SEND_BOOZ2_GPS( &booz_gps_state.ecef_pos.x,	\
			     &booz_gps_state.ecef_pos.y,	\
			     &booz_gps_state.ecef_pos.z,	\
			     &booz_gps_state.ecef_vel.x,	\
			     &booz_gps_state.ecef_vel.y,	\
			     &booz_gps_state.ecef_vel.z,	\
			     &booz_gps_state.pacc,		\
			     &booz_gps_state.sacc,		\
			     &booz_gps_state.pdop,		\
			     &booz_gps_state.num_sv,		\
			     &booz_gps_state.fix);		\
  }
#else
#define PERIODIC_SEND_BOOZ2_GPS() {}
#endif

#include "booz2_navigation.h"
#define PERIODIC_SEND_BOOZ2_NAV_REF() {					\
    DOWNLINK_SEND_BOOZ2_NAV_REF(&booz_ins_ltp_def.ecef.x, &booz_ins_ltp_def.ecef.y, &booz_ins_ltp_def.ecef.z); \
  }

#define PERIODIC_SEND_BOOZ2_NAV_STATUS() {				\
    DOWNLINK_SEND_BOOZ2_NAV_STATUS(&block_time,&stage_time,&nav_block,&nav_stage,&horizontal_mode); \
    if (horizontal_mode == HORIZONTAL_MODE_ROUTE) {			\
      float sx = POS_FLOAT_OF_BFP(waypoints[nav_segment_start].x);	\
      float sy = POS_FLOAT_OF_BFP(waypoints[nav_segment_start].y);	\
      float ex = POS_FLOAT_OF_BFP(waypoints[nav_segment_end].x);	\
      float ey = POS_FLOAT_OF_BFP(waypoints[nav_segment_end].y);	\
      DOWNLINK_SEND_SEGMENT(&sx, &sy, &ex, &ey);			\
    }									\
  }

#define PERIODIC_SEND_WP_MOVED() { \
  static uint8_t i; \
  i++; if (i >= nb_waypoint) i = 0; \
  DOWNLINK_SEND_WP_MOVED_LTP(&i, &(waypoints[i].x), &(waypoints[i].y), &(waypoints[i].z)); \
}


#define PERIODIC_SEND_BOOZ2_TUNE_HOVER() {				\
    DOWNLINK_SEND_BOOZ2_TUNE_HOVER(&radio_control.values[RADIO_CONTROL_ROLL], \
				   &radio_control.values[RADIO_CONTROL_PITCH], \
				   &radio_control.values[RADIO_CONTROL_YAW], \
				   &booz_stabilization_cmd[COMMAND_ROLL], \
				   &booz_stabilization_cmd[COMMAND_PITCH], \
				   &booz_stabilization_cmd[COMMAND_YAW], \
				   &booz_stabilization_cmd[COMMAND_THRUST], \
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
