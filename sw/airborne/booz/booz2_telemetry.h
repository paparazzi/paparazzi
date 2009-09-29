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

#ifndef TELEMETRY_STARTUP_DELAY
#define TELEMETRY_STARTUP_DELAY 0
#endif

#define PERIODIC_SEND_ALIVE(_chan) DOWNLINK_SEND_ALIVE(_chan, 16, MD5SUM)

#include "booz2_battery.h"
#include "booz_imu.h"
#include "booz2_gps.h"
#include "booz2_ins.h"
#include "booz_ahrs.h"

extern uint8_t telemetry_mode_Main_DefaultChannel;

#ifdef USE_GPS
#define PERIODIC_SEND_BOOZ_STATUS(_chan) {				\
    uint32_t booz_imu_nb_err = 0;					\
    uint8_t _twi_blmc_nb_err = 0;					\
    DOWNLINK_SEND_BOOZ_STATUS(_chan,					\
			      &booz_imu_nb_err,				\
			      &_twi_blmc_nb_err,			\
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
#define PERIODIC_SEND_BOOZ_STATUS(_chan) {				\
    uint32_t booz_imu_nb_err = 0;					\
    uint8_t twi_blmc_nb_err = 0;					\
    uint8_t  fix = BOOZ2_GPS_FIX_NONE;					\
    DOWNLINK_SEND_BOOZ_STATUS(_chan,					\
			      &booz_imu_nb_err,				\
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
#define PERIODIC_SEND_RC(_chan) DOWNLINK_SEND_RC(_chan, RADIO_CONTROL_NB_CHANNEL, radio_control.values)
#define PERIODIC_SEND_BOOZ2_RADIO_CONTROL(_chan) {			             \
    int16_t foo = 0;							             \
    DOWNLINK_SEND_BOOZ2_RADIO_CONTROL(_chan,				             \
				      &radio_control.values[RADIO_CONTROL_ROLL],     \
				      &radio_control.values[RADIO_CONTROL_PITCH],    \
				      &radio_control.values[RADIO_CONTROL_YAW],      \
				      &radio_control.values[RADIO_CONTROL_THROTTLE], \
				      &radio_control.values[RADIO_CONTROL_MODE],     \
				      &foo,				             \
				      &radio_control.status);}

#else
#define PERIODIC_SEND_RC(_chan) {}
#define PERIODIC_SEND_BOOZ2_RADIO_CONTROL(_chan) {}
#endif

#ifdef RADIO_CONTROL_TYPE_PPM
#define PERIODIC_SEND_PPM(_chan)					\
  DOWNLINK_SEND_PPM(_chan,						\
		    &radio_control.frame_rate,				\
		    RADIO_CONTROL_NB_CHANNEL,				\
		    booz_radio_control_ppm_pulses)
#else
#define PERIODIC_SEND_PPM(_chan) {}
#endif

#define PERIODIC_SEND_BOOZ2_GYRO(_chan) {		\
    DOWNLINK_SEND_BOOZ2_GYRO(_chan,			\
			     &booz_imu.gyro.p,		\
			     &booz_imu.gyro.q,		\
			     &booz_imu.gyro.r);		\
  }

#define PERIODIC_SEND_BOOZ2_ACCEL(_chan) {			\
    DOWNLINK_SEND_BOOZ2_ACCEL(_chan,				\
			      &booz_imu.accel.x,		\
			      &booz_imu.accel.y,		\
			      &booz_imu.accel.z);		\
  }

#define PERIODIC_SEND_BOOZ2_MAG(_chan) {			\
    DOWNLINK_SEND_BOOZ2_MAG(_chan,				\
			    &booz_imu.mag.x,			\
			    &booz_imu.mag.y,			\
			    &booz_imu.mag.z);			\
  }

#define PERIODIC_SEND_IMU_GYRO_RAW(_chan) {				\
    DOWNLINK_SEND_IMU_GYRO_RAW(_chan,					\
			       &booz_imu.gyro_unscaled.p,		\
			       &booz_imu.gyro_unscaled.q,		\
			       &booz_imu.gyro_unscaled.r);		\
  }

#define PERIODIC_SEND_IMU_ACCEL_RAW(_chan) {				\
    DOWNLINK_SEND_IMU_ACCEL_RAW(_chan,					\
				&booz_imu.accel_unscaled.x,		\
				&booz_imu.accel_unscaled.y,		\
				&booz_imu.accel_unscaled.z);		\
  }

#define PERIODIC_SEND_IMU_MAG_RAW(_chan) {				\
    DOWNLINK_SEND_IMU_MAG_RAW(_chan,					\
			      &booz_imu.mag_unscaled.x,			\
			      &booz_imu.mag_unscaled.y,			\
			      &booz_imu.mag_unscaled.z);		\
  }

#define PERIODIC_SEND_BOOZ2_BARO_RAW(_chan) {				\
    DOWNLINK_SEND_BOOZ2_BARO_RAW(_chan,					\
				 &booz2_analog_baro_offset,		\
			         &booz2_analog_baro_value,		\
			         &booz2_analog_baro_value_filtered);	\
  }

#include "booz_stabilization.h"
#define PERIODIC_SEND_BOOZ2_RATE_LOOP(_chan) {				    \
    DOWNLINK_SEND_BOOZ2_RATE_LOOP(_chan,				    \
				  &booz_stabilization_rate_measure.p,	    \
				  &booz_stabilization_rate_measure.q,	    \
				  &booz_stabilization_rate_measure.r,	    \
				  &booz_stabilization_rate_sp.p,	    \
				  &booz_stabilization_rate_sp.q,	    \
				  &booz_stabilization_rate_sp.r,	    \
				  &booz_stabilization_cmd[COMMAND_ROLL],    \
				  &booz_stabilization_cmd[COMMAND_PITCH],   \
				  &booz_stabilization_cmd[COMMAND_YAW],     \
				  &booz_stabilization_cmd[COMMAND_THRUST]); \
  }

#ifdef STABILISATION_ATTITUDE_TYPE_INT
#define PERIODIC_SEND_BOOZ2_STAB_ATTITUDE(_chan) {			\
    DOWNLINK_SEND_BOOZ2_STAB_ATTITUDE_INT(_chan,			\
					  &booz_ahrs.body_rate.p,	\
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


#define PERIODIC_SEND_BOOZ2_STAB_ATTITUDE_REF(_chan) {			\
    DOWNLINK_SEND_BOOZ2_STAB_ATTITUDE_REF_INT(_chan,			\
					      &booz_stab_att_sp_euler.phi, \
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
#define PERIODIC_SEND_BOOZ2_STAB_ATTITUDE(_chan) {			\
    DOWNLINK_SEND_BOOZ2_STAB_ATTITUDE_FLOAT(_chan,			\
					    &booz_ahrs_float.body_rate.p,	\
					    &booz_ahrs_float.body_rate.q,	\
					    &booz_ahrs_float.body_rate.r,	\
					    &booz_ahrs_float.ltp_to_body_euler.phi, \
					    &booz_ahrs_float.ltp_to_body_euler.theta, \
					    &booz_ahrs_float.ltp_to_body_euler.psi, \
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

#define PERIODIC_SEND_BOOZ2_STAB_ATTITUDE_REF(_chan) {			\
    DOWNLINK_SEND_BOOZ2_STAB_ATTITUDE_REF_FLOAT(_chan,			\
						&booz_stab_att_sp_euler.phi, \
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
#define PERIODIC_SEND_BOOZ2_FILTER_ALIGNER(_chan) {			\
    DOWNLINK_SEND_BOOZ2_FILTER_ALIGNER(_chan,				\
				       &booz_ahrs_aligner.lp_gyro.p,	\
				       &booz_ahrs_aligner.lp_gyro.q,	\
				       &booz_ahrs_aligner.lp_gyro.r,	\
				       &booz_imu.gyro.p,		\
				       &booz_imu.gyro.q,		\
				       &booz_imu.gyro.r,		\
				       &booz_ahrs_aligner.noise,	\
				       &booz_ahrs_aligner.low_noise_cnt); \
  }


#define PERIODIC_SEND_BOOZ2_CMD(_chan) {				\
    DOWNLINK_SEND_BOOZ2_CMD(_chan,					\
			    &booz_stabilization_cmd[COMMAND_ROLL],	\
			    &booz_stabilization_cmd[COMMAND_PITCH],	\
			    &booz_stabilization_cmd[COMMAND_YAW],	\
			    &booz_stabilization_cmd[COMMAND_THRUST]);	\
  }


#ifdef USE_AHRS_CMPL
#include "ahrs/booz2_filter_attitude_cmpl_euler.h"
#define PERIODIC_SEND_BOOZ2_FILTER(_chan) {				\
    DOWNLINK_SEND_BOOZ2_FILTER(_chan,					\
			       &booz_ahrs.ltp_to_imu_euler.phi,		\
			       &booz_ahrs.ltp_to_imu_euler.theta,	\
			       &booz_ahrs.ltp_to_imu_euler.psi,		\
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
#define PERIODIC_SEND_BOOZ2_FILTER(_chan) {}
#endif

#ifdef USE_AHRS_LKF
#include "booz_ahrs.h"
#include "ahrs/booz_ahrs_float_lkf.h"
#define PERIODIC_SEND_BOOZ_AHRS_LKF(_chan) {				\
    DOWNLINK_SEND_BOOZ_AHRS_LKF(&bafl_eulers.phi,			\
				_chan,					\
				&bafl_eulers.theta,			\
				&bafl_eulers.psi,			\
				&bafl_quat.qi,				\
				&bafl_quat.qx,				\
				&bafl_quat.qy,				\
				&bafl_quat.qz,				\
				&bafl_rates.p,				\
				&bafl_rates.q,				\
				&bafl_rates.r,				\
				&bafl_accel_measure.x,			\
				&bafl_accel_measure.y,			\
				&bafl_accel_measure.z,			\
				&bafl_mag.x,				\
				&bafl_mag.y,				\
				&bafl_mag.z);				\
  }
#define PERIODIC_SEND_BOOZ_AHRS_LKF_DEBUG(_chan) {		   \
    DOWNLINK_SEND_BOOZ_AHRS_LKF_DEBUG(_chan,			   \
				      &bafl_X[0],		   \
				      &bafl_X[1],		   \
				      &bafl_X[2],		   \
				      &bafl_bias.p,		   \
				      &bafl_bias.q,		   \
				      &bafl_bias.r,		   \
				      &bafl_qnorm,		   \
				      &bafl_phi_accel,		   \
				      &bafl_theta_accel,	   \
				      &bafl_P[0][0],		   \
				      &bafl_P[1][1],		   \
				      &bafl_P[2][2],		   \
				      &bafl_P[3][3],		   \
				      &bafl_P[4][4],		   \
				      &bafl_P[5][5]);		   \
  }
#define PERIODIC_SEND_BOOZ_AHRS_LKF_ACC_DBG(_chan) {		    \
    DOWNLINK_SEND_BOOZ_AHRS_LKF_ACC_DBG(_chan,			    \
					&bafl_q_a_err.qi,	    \
					&bafl_q_a_err.qx,	    \
					&bafl_q_a_err.qy,	    \
					&bafl_q_a_err.qz,	    \
					&bafl_b_a_err.p,	    \
					&bafl_b_a_err.q,	    \
					&bafl_b_a_err.r);	    \
  }
#define PERIODIC_SEND_BOOZ_AHRS_LKF_MAG_DBG(_chan) {	    \
    DOWNLINK_SEND_BOOZ_AHRS_LKF_MAG_DBG(_chan,		    \
					&bafl_q_m_err.qi,   \
					&bafl_q_m_err.qx,   \
					&bafl_q_m_err.qy,   \
					&bafl_q_m_err.qz,   \
					&bafl_b_m_err.p,    \
					&bafl_b_m_err.q,    \
					&bafl_b_m_err.r);   \
  }
#else
#define PERIODIC_SEND_BOOZ_AHRS_LKF(_chan) {}
#define PERIODIC_SEND_BOOZ_AHRS_LKF_DEBUG(_chan) {}
#define PERIODIC_SEND_BOOZ_AHRS_LKF_MAG_DBG(_chan) {}
#define PERIODIC_SEND_BOOZ_AHRS_LKF_ACC_DBG(_chan) {}
#endif


#define PERIODIC_SEND_BOOZ2_AHRS_QUAT(_chan) {				\
    DOWNLINK_SEND_BOOZ2_AHRS_QUAT(_chan,				\
				  &booz_ahrs.ltp_to_imu_quat.qi,	\
				  &booz_ahrs.ltp_to_imu_quat.qx,	\
				  &booz_ahrs.ltp_to_imu_quat.qy,	\
				  &booz_ahrs.ltp_to_imu_quat.qz,	\
				  &booz_ahrs.ltp_to_body_quat.qi,	\
				  &booz_ahrs.ltp_to_body_quat.qx,	\
				  &booz_ahrs.ltp_to_body_quat.qy,	\
				  &booz_ahrs.ltp_to_body_quat.qz);	\
  }

#define PERIODIC_SEND_BOOZ2_AHRS_EULER(_chan) {				\
    DOWNLINK_SEND_BOOZ2_AHRS_EULER(_chan,				\
				   &booz_ahrs.ltp_to_imu_euler.phi,	\
				   &booz_ahrs.ltp_to_imu_euler.theta,	\
				   &booz_ahrs.ltp_to_imu_euler.psi,	\
				   &booz_ahrs.ltp_to_body_euler.phi,	\
				   &booz_ahrs.ltp_to_body_euler.theta,	\
				   &booz_ahrs.ltp_to_body_euler.psi);	\
  }

#define PERIODIC_SEND_BOOZ2_AHRS_RMAT(_chan) {				\
    DOWNLINK_SEND_BOOZ2_AHRS_RMAT(_chan,				\
				  &booz_ahrs.ltp_to_imu_rmat.m[0],	\
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




#define PERIODIC_SEND_BOOZ2_FILTER_Q(_chan) {				\
    DOWNLINK_SEND_BOOZ2_FILTER_Q(_chan,					\
				 &booz2_filter_attitude_quat.qi,	\
				 &booz2_filter_attitude_quat.qx,	\
				 &booz2_filter_attitude_quat.qy,	\
				 &booz2_filter_attitude_quat.qz);	\
  }

#ifdef USE_VFF
#include "ins/booz2_vf_float.h"
#define PERIODIC_SEND_BOOZ2_VFF(_chan) {		\
    DOWNLINK_SEND_BOOZ2_VFF(_chan,			\
			    &b2_vff_z_meas,		\
			    &b2_vff_z,			\
			    &b2_vff_zdot,		\
			    &b2_vff_bias,		\
			    & b2_vff_P[0][0],		\
			    & b2_vff_P[1][1],		\
			    & b2_vff_P[2][2]);		\
  }
#else
#define PERIODIC_SEND_BOOZ2_VFF(_chan) {}
#endif

#ifdef USE_HFF
#include "ins/booz2_hf_float.h"
#define PERIODIC_SEND_BOOZ2_HFF(_chan) {	\
    DOWNLINK_SEND_BOOZ2_HFF(_chan,		\
                            &b2_hff_state.x,			\
                            &b2_hff_state.y,			\
                            &b2_hff_state.xdot,         \
                            &b2_hff_state.ydot,			\
                            &b2_hff_state.xdotdot,      \
                            &b2_hff_state.ydotdot);     \
  }
#define PERIODIC_SEND_BOOZ2_HFF_DBG(_chan) {                \
	DOWNLINK_SEND_BOOZ2_HFF_DBG(_chan,                      \
                                &b2_hff_x_meas,             \
                                &b2_hff_y_meas,             \
                                &b2_hff_xd_meas,            \
                                &b2_hff_yd_meas,            \
                                &b2_hff_state.xP[0][0],     \
                                &b2_hff_state.yP[0][0],     \
                                &b2_hff_state.xP[1][1],     \
                                &b2_hff_state.yP[1][1]);    \
  }
#ifdef GPS_LAG
#define PERIODIC_SEND_BOOZ2_HFF_GPS(_chan) {	\
    DOWNLINK_SEND_BOOZ2_HFF_GPS(_chan,			\
							  &b2_hff_rb_last->lag_counter,		\
							  &lag_counter_err,	\
							  &save_counter);	\
  }
#else
#define PERIODIC_SEND_BOOZ2_HFF_GPS(_chan) {}
#endif
#else
#define PERIODIC_SEND_BOOZ2_HFF(_chan) {}
#define PERIODIC_SEND_BOOZ2_HFF_DBG(_chan) {}
#define PERIODIC_SEND_BOOZ2_HFF_GPS(_chan) {}
#endif

#define PERIODIC_SEND_BOOZ2_GUIDANCE(_chan) {				\
    DOWNLINK_SEND_BOOZ2_GUIDANCE(_chan,					\
				 &booz2_guidance_h_cur_pos.x,		\
				 &booz2_guidance_h_cur_pos.y,		\
				 &booz2_guidance_h_held_pos.x,		\
				 &booz2_guidance_h_held_pos.y);		\
  }

#define PERIODIC_SEND_BOOZ2_INS(_chan) {				\
    DOWNLINK_SEND_BOOZ2_INS(_chan,					\
			    &booz_ins_baro_alt,				\
			    &booz_ins_ltp_pos.z,			\
			    &booz_ins_ltp_speed.z,			\
			    &booz_ins_ltp_accel.z);			\
  }


#define PERIODIC_SEND_BOOZ2_INS2(_chan) {			\
    struct Int32Vect3 pos_low_res;				\
    pos_low_res.x = (int32_t)(b2ins_pos_ltp.x>>20);		\
    pos_low_res.y = (int32_t)(b2ins_pos_ltp.y>>20);		\
    pos_low_res.z = (int32_t)(b2ins_pos_ltp.z>>20);		\
    DOWNLINK_SEND_BOOZ2_INS2(_chan,				\
			     &b2ins_accel_ltp.x,		\
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
#include "ins/booz2_hf_float.h"
#define PERIODIC_SEND_BOOZ2_INS3(_chan) {				\
    DOWNLINK_SEND_BOOZ2_INS3(_chan,					\
			     &b2ins_meas_gps_pos_ned.x,			\
			     &b2ins_meas_gps_pos_ned.y,			\
			     &b2ins_meas_gps_pos_ned.z,			\
			     &b2ins_meas_gps_speed_ned.x,		\
			     &b2ins_meas_gps_speed_ned.y,		\
			     &b2ins_meas_gps_speed_ned.z		\
			     );						\
  }
#else /* !USE_GPS */
#define PERIODIC_SEND_BOOZ2_INS3(_chan) {}
#endif /* USE_GPS */

#define PERIODIC_SEND_BOOZ_INS(_chan) {			\
    DOWNLINK_SEND_BOOZ_INS(_chan,				\
					   &booz_ins_ltp_pos.x,		\
					   &booz_ins_ltp_pos.y,	    \
					   &booz_ins_ltp_pos.z,		\
					   &booz_ins_ltp_speed.x,	\
					   &booz_ins_ltp_speed.y,	\
					   &booz_ins_ltp_speed.z,	\
					   &booz_ins_ltp_accel.x,	\
					   &booz_ins_ltp_accel.y,	\
					   &booz_ins_ltp_accel.z);	\
  }

#define PERIODIC_SEND_BOOZ2_INS_REF(_chan) {				\
    DOWNLINK_SEND_BOOZ2_INS_REF(_chan,					\
				&booz_ins_ltp_def.ecef.x,		\
				&booz_ins_ltp_def.ecef.y,		\
				&booz_ins_ltp_def.ecef.z,		\
				&booz_ins_ltp_def.lla.lat,		\
				&booz_ins_ltp_def.lla.lon,		\
				&booz_ins_ltp_def.lla.alt,		\
				&booz_ins_ltp_def.hmsl,		\
				&booz_ins_qfe);				\
  }



#define PERIODIC_SEND_BOOZ2_VERT_LOOP(_chan) {				\
    DOWNLINK_SEND_BOOZ2_VERT_LOOP(_chan,				\
				  &booz2_guidance_v_z_sp,		\
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

#define PERIODIC_SEND_BOOZ2_HOVER_LOOP(_chan) {				\
    DOWNLINK_SEND_BOOZ2_HOVER_LOOP(_chan,				\
				   &booz2_guidance_h_pos_sp.x,		\
				   &booz2_guidance_h_pos_sp.y,		\
				   &booz_ins_ltp_pos.x,			\
				   &booz_ins_ltp_pos.y,			\
				   &booz_ins_ltp_speed.x,		\
				   &booz_ins_ltp_speed.y,		\
				   &booz_ins_ltp_accel.x,		\
				   &booz_ins_ltp_accel.y,		\
				   &booz2_guidance_h_pos_err.x,		\
				   &booz2_guidance_h_pos_err.y,		\
				   &booz2_guidance_h_speed_err.x,	\
				   &booz2_guidance_h_speed_err.y,	\
				   &booz2_guidance_h_pos_err_sum.x,	\
				   &booz2_guidance_h_pos_err_sum.y,	\
				   &booz2_guidance_h_nav_err.x,	\
				   &booz2_guidance_h_nav_err.y,	\
				   &booz2_guidance_h_command_earth.x,	\
				   &booz2_guidance_h_command_earth.y,	\
				   &booz2_guidance_h_command_body.phi,	\
				   &booz2_guidance_h_command_body.theta, \
				   &booz2_guidance_h_command_body.psi);	\
  }


#include "booz2_gps.h"
#include "booz2_navigation.h"
#define PERIODIC_SEND_BOOZ2_FP(_chan) {					\
    int32_t carrot_up = -booz2_guidance_v_z_sp;				\
    DOWNLINK_SEND_BOOZ2_FP( _chan,					\
			    &booz_ins_enu_pos.x,			\
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
			    &booz_stabilization_cmd[COMMAND_THRUST], \
          &booz2_autopilot_flight_time);	\
  }

#ifdef USE_GPS
#define PERIODIC_SEND_BOOZ2_GPS(_chan) {				\
    DOWNLINK_SEND_BOOZ2_GPS( _chan,					\
			     &booz_gps_state.ecef_pos.x,		\
			     &booz_gps_state.ecef_pos.y,		\
			     &booz_gps_state.ecef_pos.z,		\
			     &booz_gps_state.lla_pos.lat,		\
			     &booz_gps_state.lla_pos.lon,		\
			     &booz_gps_state.lla_pos.alt,		\
			     &booz_gps_state.ecef_vel.x,		\
			     &booz_gps_state.ecef_vel.y,		\
			     &booz_gps_state.ecef_vel.z,		\
			     &booz_gps_state.pacc,			\
			     &booz_gps_state.sacc,			\
			     &booz_gps_state.pdop,			\
			     &booz_gps_state.num_sv,			\
			     &booz_gps_state.fix);			\
  }
#else
#define PERIODIC_SEND_BOOZ2_GPS(_chan) {}
#endif

#include "booz2_navigation.h"
#define PERIODIC_SEND_BOOZ2_NAV_STATUS(_chan) {				\
    DOWNLINK_SEND_BOOZ2_NAV_STATUS(_chan,				\
				   &block_time,				\
				   &stage_time,				\
				   &nav_block,				\
				   &nav_stage,				\
				   &horizontal_mode);			\
    if (horizontal_mode == HORIZONTAL_MODE_ROUTE) {			\
      float sx = POS_FLOAT_OF_BFP(waypoints[nav_segment_start].x);	\
      float sy = POS_FLOAT_OF_BFP(waypoints[nav_segment_start].y);	\
      float ex = POS_FLOAT_OF_BFP(waypoints[nav_segment_end].x);	\
      float ey = POS_FLOAT_OF_BFP(waypoints[nav_segment_end].y);	\
      DOWNLINK_SEND_SEGMENT(_chan, &sx, &sy, &ex, &ey);			\
    }									\
    else if (horizontal_mode == HORIZONTAL_MODE_CIRCLE) {			\
      float cx = POS_FLOAT_OF_BFP(waypoints[nav_circle_centre].x);	\
      float cy = POS_FLOAT_OF_BFP(waypoints[nav_circle_centre].y);	\
      float r = POS_FLOAT_OF_BFP(nav_circle_radius); \
      DOWNLINK_SEND_CIRCLE(_chan, &cx, &cy, &r);			\
    }									\
  }

#define PERIODIC_SEND_WP_MOVED(_chan) {					\
    static uint8_t i;							\
    i++; if (i >= nb_waypoint) i = 0;					\
    DOWNLINK_SEND_WP_MOVED_ENU(_chan,					\
			       &i,					\
			       &(waypoints[i].x),			\
			       &(waypoints[i].y),			\
			       &(waypoints[i].z));			\
  }

#ifdef USE_CAM
#include "booz2_cam.h"
#define PERIODIC_SEND_BOOZ2_CAM(_chan) DOWNLINK_SEND_BOOZ2_CAM(_chan,&booz2_cam_tilt,&booz2_cam_pan);
#else
#define PERIODIC_SEND_BOOZ2_CAM(_chan) {}
#endif

#define PERIODIC_SEND_BOOZ2_TUNE_HOVER(_chan) {				       \
    DOWNLINK_SEND_BOOZ2_TUNE_HOVER(_chan,				       \
				   &radio_control.values[RADIO_CONTROL_ROLL],  \
				   &radio_control.values[RADIO_CONTROL_PITCH], \
				   &radio_control.values[RADIO_CONTROL_YAW],   \
				   &booz_stabilization_cmd[COMMAND_ROLL],      \
				   &booz_stabilization_cmd[COMMAND_PITCH],     \
				   &booz_stabilization_cmd[COMMAND_YAW],       \
				   &booz_stabilization_cmd[COMMAND_THRUST],    \
				   &booz_ahrs.ltp_to_imu_euler.phi,	       \
				   &booz_ahrs.ltp_to_imu_euler.theta,	       \
				   &booz_ahrs.ltp_to_imu_euler.psi,	       \
				   &booz_ahrs.ltp_to_body_euler.phi,	       \
				   &booz_ahrs.ltp_to_body_euler.theta,	       \
				   &booz_ahrs.ltp_to_body_euler.psi	       \
				   );					       \
  }

#ifdef BOOZ2_SONAR
#include "booz2_sonar.h"
#define PERIODIC_SEND_BOOZ2_SONAR(_chan) DOWNLINK_SEND_BOOZ2_SONAR(_chan,&booz2_sonar_front,&booz2_sonar_back,&booz2_sonar_right,&booz2_sonar_left);
#else
#define PERIODIC_SEND_BOOZ2_SONAR(_chan) {}
#endif


#include "settings.h"
#define PERIODIC_SEND_DL_VALUE(_chan) PeriodicSendDlValue(_chan)

#include "periodic.h"
#define Booz2TelemetryPeriodic() {			\
    PeriodicSendMain_DefaultChannel();			\
  }


#endif /* BOOZ2_TELEMETRY_H */
