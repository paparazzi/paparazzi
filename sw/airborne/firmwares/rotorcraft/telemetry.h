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

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "std.h"
#include "messages.h"
#include "mcu_periph/uart.h"

#include "downlink.h"

#ifdef RADIO_CONTROL
#include "subsystems/radio_control.h"
#endif

#include "firmwares/rotorcraft/autopilot.h"
#include "firmwares/rotorcraft/guidance.h"

#include "firmwares/rotorcraft/actuators.h"

#ifndef TELEMETRY_STARTUP_DELAY
#define TELEMETRY_STARTUP_DELAY 0
#endif

#define PERIODIC_SEND_ALIVE(_chan) DOWNLINK_SEND_ALIVE(_chan, 16, MD5SUM)

#include "subsystems/electrical.h"
#include "subsystems/imu.h"
#include "booz_gps.h"
#include "subsystems/ins.h"
#include "subsystems/ahrs.h"
//FIXME: wtf ??!!
#include "mcu_periph/i2c_arch.h"

extern uint8_t telemetry_mode_Main_DefaultChannel;

#ifdef USE_GPS
#define PERIODIC_SEND_ROTORCRAFT_STATUS(_chan) {			\
    uint32_t imu_nb_err = 0;						\
    uint8_t _twi_blmc_nb_err = 0;					\
    DOWNLINK_SEND_ROTORCRAFT_STATUS(_chan,				\
				    &imu_nb_err,			\
				    &_twi_blmc_nb_err,			\
				    &radio_control.status,		\
				    &radio_control.frame_rate,		\
				    &booz_gps_state.fix,		\
				    &autopilot_mode,			\
				    &autopilot_in_flight,		\
				    &autopilot_motors_on,		\
				    &guidance_h_mode,			\
				    &guidance_v_mode,			\
				    &electrical.vsupply,		\
				    &cpu_time_sec			\
				    );					\
  }
#else /* !USE_GPS */
#define PERIODIC_SEND_ROTORCRAFT_STATUS(_chan) {			\
    uint32_t imu_nb_err = 0;						\
    uint8_t twi_blmc_nb_err = 0;					\
    uint8_t  fix = BOOZ2_GPS_FIX_NONE;					\
    DOWNLINK_SEND_ROTORCRAFT_STATUS(_chan,				\
				    &imu_nb_err,			\
				    &twi_blmc_nb_err,			\
				    &radio_control.status,		\
				    &radio_control.frame_rate,		\
				    &fix,				\
				    &autopilot_mode,			\
				    &autopilot_in_flight,		\
				    &autopilot_motors_on,		\
				    &guidance_h_mode,			\
				    &guidance_v_mode,			\
				    &electrical.vsupply,		\
				    &cpu_time_sec			\
				    );					\
  }
#endif /* USE_GPS */

#ifdef RADIO_CONTROL
#define PERIODIC_SEND_RC(_chan) DOWNLINK_SEND_RC(_chan, RADIO_CONTROL_NB_CHANNEL, radio_control.values)
#if defined RADIO_KILL_SWITCH
#define PERIODIC_SEND_BOOZ2_RADIO_CONTROL(_chan) SEND_BOOZ2_RADIO_CONTROL( _chan, &radio_control.values[RADIO_KILL_SWITCH])
#else /* ! RADIO_KILL_SWITCH */
#define PERIODIC_SEND_BOOZ2_RADIO_CONTROL(_chan) {			                    \
    int16_t foo = -42;							                    \
    SEND_BOOZ2_RADIO_CONTROL( _chan, &foo)				                    \
}
#endif /* !RADIO_KILL_SWITCH */
#define SEND_BOOZ2_RADIO_CONTROL(_chan, _kill_switch) {			                    \
    DOWNLINK_SEND_BOOZ2_RADIO_CONTROL(_chan,				                    \
				      &radio_control.values[RADIO_ROLL],            \
				      &radio_control.values[RADIO_PITCH],           \
				      &radio_control.values[RADIO_YAW],	            \
				      &radio_control.values[RADIO_THROTTLE],        \
				      &radio_control.values[RADIO_MODE],            \
				      _kill_switch,                                        \
				      &radio_control.status);}
#else /* ! RADIO_CONTROL */
#define PERIODIC_SEND_RC(_chan) {}
#define PERIODIC_SEND_BOOZ2_RADIO_CONTROL(_chan) {}
#endif

#ifdef RADIO_CONTROL_TYPE_PPM
#define PERIODIC_SEND_PPM(_chan)                    \
  DOWNLINK_SEND_PPM(_chan,                          \
                    &radio_control.frame_rate,      \
                    PPM_NB_CHANNEL,                 \
                    ppm_pulses)
#else
#define PERIODIC_SEND_PPM(_chan) {}
#endif

#define PERIODIC_SEND_IMU_GYRO_SCALED(_chan) {		\
    DOWNLINK_SEND_IMU_GYRO_SCALED(_chan,			\
			     &imu.gyro.p,		\
			     &imu.gyro.q,		\
			     &imu.gyro.r);		\
  }

#define PERIODIC_SEND_IMU_ACCEL_SCALED(_chan) {			\
    DOWNLINK_SEND_IMU_ACCEL_SCALED(_chan,				\
			      &imu.accel.x,		\
			      &imu.accel.y,		\
			      &imu.accel.z);		\
  }

#define PERIODIC_SEND_IMU_MAG_SCALED(_chan) {			\
    DOWNLINK_SEND_IMU_MAG_SCALED(_chan,				\
			    &imu.mag.x,			\
			    &imu.mag.y,			\
			    &imu.mag.z);			\
  }

#define PERIODIC_SEND_IMU_GYRO_RAW(_chan) {				\
    DOWNLINK_SEND_IMU_GYRO_RAW(_chan,					\
			       &imu.gyro_unscaled.p,		\
			       &imu.gyro_unscaled.q,		\
			       &imu.gyro_unscaled.r);		\
  }

#define PERIODIC_SEND_IMU_ACCEL_RAW(_chan) {				\
    DOWNLINK_SEND_IMU_ACCEL_RAW(_chan,					\
				&imu.accel_unscaled.x,		\
				&imu.accel_unscaled.y,		\
				&imu.accel_unscaled.z);		\
  }

#define PERIODIC_SEND_IMU_MAG_RAW(_chan) {				\
    DOWNLINK_SEND_IMU_MAG_RAW(_chan,					\
			      &imu.mag_unscaled.x,			\
			      &imu.mag_unscaled.y,			\
			      &imu.mag_unscaled.z);		\
  }

/* FIXME: make that depend on board */
#define PERIODIC_SEND_BOOZ_BARO_RAW(_chan) {				\
    DOWNLINK_SEND_BOOZ2_BARO_RAW(_chan,					\
				 &baro_board.offset,			\
			         &baro.absolute,			\
			         &baro_board.value_filtered);		\
  }


#define PERIODIC_SEND_BARO_RAW(_chan) {					\
    DOWNLINK_SEND_BARO_RAW(_chan,					\
			   &baro.absolute,				\
			   &baro.differential);				\
  }




#include "firmwares/rotorcraft/stabilization.h"
#define PERIODIC_SEND_RATE_LOOP(_chan) {                          \
    DOWNLINK_SEND_RATE_LOOP(_chan,                                \
                                  &stabilization_rate_sp.p,        \
                                  &stabilization_rate_sp.q,        \
                                  &stabilization_rate_sp.r,        \
                                  &stabilization_rate_ref.p,       \
                                  &stabilization_rate_ref.q,       \
                                  &stabilization_rate_ref.r,       \
                                  &stabilization_rate_refdot.p,    \
                                  &stabilization_rate_refdot.q,    \
                                  &stabilization_rate_refdot.r,    \
                                  &stabilization_rate_sum_err.p,    \
                                  &stabilization_rate_sum_err.q,    \
                                  &stabilization_rate_sum_err.r,    \
                                  &stabilization_rate_ff_cmd.p,    \
                                  &stabilization_rate_ff_cmd.q,    \
                                  &stabilization_rate_ff_cmd.r,    \
                                  &stabilization_rate_fb_cmd.p,    \
                                  &stabilization_rate_fb_cmd.q,    \
                                  &stabilization_rate_fb_cmd.r,    \
                                  &stabilization_cmd[COMMAND_THRUST]); \
  }

#ifdef STABILISATION_ATTITUDE_TYPE_INT
#define PERIODIC_SEND_STAB_ATTITUDE(_chan) {			\
    DOWNLINK_SEND_STAB_ATTITUDE_INT(_chan,			\
					  &ahrs.body_rate.p,	\
					  &ahrs.body_rate.q,	\
					  &ahrs.body_rate.r,	\
					  &ahrs.ltp_to_body_euler.phi, \
					  &ahrs.ltp_to_body_euler.theta, \
					  &ahrs.ltp_to_body_euler.psi, \
					  &stab_att_sp_euler.phi, \
					  &stab_att_sp_euler.theta, \
					  &stab_att_sp_euler.psi, \
					  &stabilization_att_sum_err.phi, \
					  &stabilization_att_sum_err.theta, \
					  &stabilization_att_sum_err.psi, \
					  &stabilization_att_fb_cmd[COMMAND_ROLL], \
					  &stabilization_att_fb_cmd[COMMAND_PITCH], \
					  &stabilization_att_fb_cmd[COMMAND_YAW], \
					  &stabilization_att_ff_cmd[COMMAND_ROLL], \
					  &stabilization_att_ff_cmd[COMMAND_PITCH], \
					  &stabilization_att_ff_cmd[COMMAND_YAW], \
					  &stabilization_cmd[COMMAND_ROLL], \
					  &stabilization_cmd[COMMAND_PITCH], \
					  &stabilization_cmd[COMMAND_YAW]); \
  }


#define PERIODIC_SEND_STAB_ATTITUDE_REF(_chan) {			\
    DOWNLINK_SEND_STAB_ATTITUDE_REF_INT(_chan,			\
					      &stab_att_sp_euler.phi, \
					      &stab_att_sp_euler.theta, \
					      &stab_att_sp_euler.psi, \
					      &stab_att_ref_euler.phi, \
					      &stab_att_ref_euler.theta, \
					      &stab_att_ref_euler.psi, \
					      &stab_att_ref_rate.p, \
					      &stab_att_ref_rate.q, \
					      &stab_att_ref_rate.r, \
					      &stab_att_ref_accel.p, \
					      &stab_att_ref_accel.q, \
					      &stab_att_ref_accel.r); \
  }
#endif /* STABILISATION_ATTITUDE_TYPE_INT */

#ifdef STABILISATION_ATTITUDE_TYPE_FLOAT
#define PERIODIC_SEND_STAB_ATTITUDE(_chan) {			\
    DOWNLINK_SEND_STAB_ATTITUDE_FLOAT(_chan,			\
					    &ahrs_float.body_rate.p,	\
					    &ahrs_float.body_rate.q,	\
					    &ahrs_float.body_rate.r,	\
					    &ahrs_float.ltp_to_body_euler.phi, \
					    &ahrs_float.ltp_to_body_euler.theta, \
					    &ahrs_float.ltp_to_body_euler.psi, \
					    &stab_att_ref_euler.phi, \
					    &stab_att_ref_euler.theta, \
					    &stab_att_ref_euler.psi, \
					    &stabilization_att_sum_err.phi, \
					    &stabilization_att_sum_err.theta, \
					    &stabilization_att_sum_err.psi, \
					    &stabilization_att_fb_cmd[COMMAND_ROLL], \
					    &stabilization_att_fb_cmd[COMMAND_PITCH], \
					    &stabilization_att_fb_cmd[COMMAND_YAW], \
					    &stabilization_att_ff_cmd[COMMAND_ROLL], \
					    &stabilization_att_ff_cmd[COMMAND_PITCH], \
					    &stabilization_att_ff_cmd[COMMAND_YAW], \
					    &stabilization_cmd[COMMAND_ROLL], \
					    &stabilization_cmd[COMMAND_PITCH], \
					    &stabilization_cmd[COMMAND_YAW]); \
  }

#define PERIODIC_SEND_STAB_ATTITUDE_REF(_chan) {			\
    DOWNLINK_SEND_STAB_ATTITUDE_REF_FLOAT(_chan,			\
						&stab_att_sp_euler.phi, \
						&stab_att_sp_euler.theta, \
						&stab_att_sp_euler.psi, \
						&stab_att_ref_euler.phi, \
						&stab_att_ref_euler.theta, \
						&stab_att_ref_euler.psi, \
						&stab_att_ref_rate.p,	\
						&stab_att_ref_rate.q,	\
						&stab_att_ref_rate.r,	\
						&stab_att_ref_accel.p, \
						&stab_att_ref_accel.q, \
						&stab_att_ref_accel.r); \
  }

#endif /* STABILISATION_ATTITUDE_TYPE_FLOAT */


#include "subsystems/ahrs/ahrs_aligner.h"
#define PERIODIC_SEND_FILTER_ALIGNER(_chan) {			\
    DOWNLINK_SEND_FILTER_ALIGNER(_chan,				\
				       &ahrs_aligner.lp_gyro.p,	\
				       &ahrs_aligner.lp_gyro.q,	\
				       &ahrs_aligner.lp_gyro.r,	\
				       &imu.gyro.p,		\
				       &imu.gyro.q,		\
				       &imu.gyro.r,		\
				       &ahrs_aligner.noise,	\
				       &ahrs_aligner.low_noise_cnt); \
  }


#define PERIODIC_SEND_BOOZ2_CMD(_chan) {				\
    DOWNLINK_SEND_BOOZ2_CMD(_chan,					\
			    &stabilization_cmd[COMMAND_ROLL],	\
			    &stabilization_cmd[COMMAND_PITCH],	\
			    &stabilization_cmd[COMMAND_YAW],	\
			    &stabilization_cmd[COMMAND_THRUST]);	\
  }


#ifdef USE_AHRS_CMPL
#include "subsystems/ahrs/ahrs_int_cmpl_euler.h"
#define PERIODIC_SEND_FILTER(_chan) {					\
    DOWNLINK_SEND_FILTER(_chan,						\
			 &ahrs.ltp_to_imu_euler.phi,			\
			 &ahrs.ltp_to_imu_euler.theta,			\
			 &ahrs.ltp_to_imu_euler.psi,			\
			 &ahrs_impl.measure.phi,			\
			 &ahrs_impl.measure.theta,			\
			 &ahrs_impl.measure.psi,			\
			 &ahrs_impl.hi_res_euler.phi,			\
			 &ahrs_impl.hi_res_euler.theta,			\
			 &ahrs_impl.hi_res_euler.psi,			\
			 &ahrs_impl.residual.phi,			\
			 &ahrs_impl.residual.theta,			\
			 &ahrs_impl.residual.psi,			\
			 &ahrs_impl.gyro_bias.p,			\
			 &ahrs_impl.gyro_bias.q,			\
			 &ahrs_impl.gyro_bias.r);			\
  }
#else
#define PERIODIC_SEND_FILTER(_chan) {}
#endif

#ifdef USE_AHRS_LKF
#include "subsystems/ahrs.h"
#include "ahrs/ahrs_float_lkf.h"
#define PERIODIC_SEND_AHRS_LKF(_chan) {				\
    DOWNLINK_SEND_AHRS_LKF(&bafl_eulers.phi,			\
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
#define PERIODIC_SEND_AHRS_LKF_DEBUG(_chan) {		   \
    DOWNLINK_SEND_AHRS_LKF_DEBUG(_chan,			   \
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
#define PERIODIC_SEND_AHRS_LKF_ACC_DBG(_chan) {		    \
    DOWNLINK_SEND_AHRS_LKF_ACC_DBG(_chan,			    \
					&bafl_q_a_err.qi,	    \
					&bafl_q_a_err.qx,	    \
					&bafl_q_a_err.qy,	    \
					&bafl_q_a_err.qz,	    \
					&bafl_b_a_err.p,	    \
					&bafl_b_a_err.q,	    \
					&bafl_b_a_err.r);	    \
  }
#define PERIODIC_SEND_AHRS_LKF_MAG_DBG(_chan) {	    \
    DOWNLINK_SEND_AHRS_LKF_MAG_DBG(_chan,		    \
					&bafl_q_m_err.qi,   \
					&bafl_q_m_err.qx,   \
					&bafl_q_m_err.qy,   \
					&bafl_q_m_err.qz,   \
					&bafl_b_m_err.p,    \
					&bafl_b_m_err.q,    \
					&bafl_b_m_err.r);   \
  }
#else
#define PERIODIC_SEND_AHRS_LKF(_chan) {}
#define PERIODIC_SEND_AHRS_LKF_DEBUG(_chan) {}
#define PERIODIC_SEND_AHRS_LKF_MAG_DBG(_chan) {}
#define PERIODIC_SEND_AHRS_LKF_ACC_DBG(_chan) {}
#endif


#define PERIODIC_SEND_BOOZ2_AHRS_QUAT(_chan) {				\
    DOWNLINK_SEND_BOOZ2_AHRS_QUAT(_chan,				\
				  &ahrs.ltp_to_imu_quat.qi,	\
				  &ahrs.ltp_to_imu_quat.qx,	\
				  &ahrs.ltp_to_imu_quat.qy,	\
				  &ahrs.ltp_to_imu_quat.qz,	\
				  &ahrs.ltp_to_body_quat.qi,	\
				  &ahrs.ltp_to_body_quat.qx,	\
				  &ahrs.ltp_to_body_quat.qy,	\
				  &ahrs.ltp_to_body_quat.qz);	\
  }

#define PERIODIC_SEND_BOOZ2_AHRS_EULER(_chan) {				\
    DOWNLINK_SEND_BOOZ2_AHRS_EULER(_chan,				\
				   &ahrs.ltp_to_imu_euler.phi,	\
				   &ahrs.ltp_to_imu_euler.theta,	\
				   &ahrs.ltp_to_imu_euler.psi,	\
				   &ahrs.ltp_to_body_euler.phi,	\
				   &ahrs.ltp_to_body_euler.theta,	\
				   &ahrs.ltp_to_body_euler.psi);	\
  }

#define PERIODIC_SEND_BOOZ2_AHRS_RMAT(_chan) {				\
    DOWNLINK_SEND_BOOZ2_AHRS_RMAT(_chan,				\
				  &ahrs.ltp_to_imu_rmat.m[0],	\
				  &ahrs.ltp_to_imu_rmat.m[1],	\
				  &ahrs.ltp_to_imu_rmat.m[2],	\
				  &ahrs.ltp_to_imu_rmat.m[3],	\
				  &ahrs.ltp_to_imu_rmat.m[4],	\
				  &ahrs.ltp_to_imu_rmat.m[5],	\
				  &ahrs.ltp_to_imu_rmat.m[6],	\
				  &ahrs.ltp_to_imu_rmat.m[7],	\
				  &ahrs.ltp_to_imu_rmat.m[8],	\
				  &ahrs.ltp_to_body_rmat.m[0],	\
				  &ahrs.ltp_to_body_rmat.m[1],	\
				  &ahrs.ltp_to_body_rmat.m[2],	\
				  &ahrs.ltp_to_body_rmat.m[3],	\
				  &ahrs.ltp_to_body_rmat.m[4],	\
				  &ahrs.ltp_to_body_rmat.m[5],	\
				  &ahrs.ltp_to_body_rmat.m[6],	\
				  &ahrs.ltp_to_body_rmat.m[7],	\
				  &ahrs.ltp_to_body_rmat.m[8]);	\
  }




#define PERIODIC_SEND_FILTER_Q(_chan) {				\
    DOWNLINK_SEND_FILTER_Q(_chan,					\
				 &booz2_filter_attitude_quat.qi,	\
				 &booz2_filter_attitude_quat.qx,	\
				 &booz2_filter_attitude_quat.qy,	\
				 &booz2_filter_attitude_quat.qz);	\
  }

#ifdef USE_VFF
#include "subsystems/ins/vf_float.h"
#define PERIODIC_SEND_VFF(_chan) {		\
    DOWNLINK_SEND_VFF(_chan,			\
			    &vff_z_meas,		\
			    &vff_z,			\
			    &vff_zdot,		\
			    &vff_bias,		\
			    & vff_P[0][0],		\
			    & vff_P[1][1],		\
			    & vff_P[2][2]);		\
  }
#else
#define PERIODIC_SEND_VFF(_chan) {}
#endif

#ifdef USE_HFF
#include  "subsystems/ins/hf_float.h"
#define PERIODIC_SEND_HFF(_chan) {	\
    DOWNLINK_SEND_HFF(_chan,		\
                            &b2_hff_state.x,			\
                            &b2_hff_state.y,			\
                            &b2_hff_state.xdot,         \
                            &b2_hff_state.ydot,			\
                            &b2_hff_state.xdotdot,      \
                            &b2_hff_state.ydotdot);     \
  }
#define PERIODIC_SEND_HFF_DBG(_chan) {                \
	DOWNLINK_SEND_HFF_DBG(_chan,                      \
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
#define PERIODIC_SEND_HFF_GPS(_chan) {	\
    DOWNLINK_SEND_HFF_GPS(_chan,			\
							  &b2_hff_rb_last->lag_counter,		\
							  &lag_counter_err,	\
							  &save_counter);	\
  }
#else
#define PERIODIC_SEND_HFF_GPS(_chan) {}
#endif
#else
#define PERIODIC_SEND_HFF(_chan) {}
#define PERIODIC_SEND_HFF_DBG(_chan) {}
#define PERIODIC_SEND_HFF_GPS(_chan) {}
#endif

#define PERIODIC_SEND_GUIDANCE(_chan) {				\
    DOWNLINK_SEND_GUIDANCE(_chan,					\
				 &guidance_h_cur_pos.x,		\
				 &guidance_h_cur_pos.y,		\
				 &guidance_h_held_pos.x,		\
				 &guidance_h_held_pos.y);		\
  }

#define PERIODIC_SEND_INS_Z(_chan) {				\
    DOWNLINK_SEND_INS_Z(_chan,					\
			    &ins_baro_alt,				\
			    &ins_ltp_pos.z,			\
			    &ins_ltp_speed.z,			\
			    &ins_ltp_accel.z);			\
  }

#define PERIODIC_SEND_INS(_chan) {			\
    DOWNLINK_SEND_INS(_chan,				\
					   &ins_ltp_pos.x,		\
					   &ins_ltp_pos.y,	    \
					   &ins_ltp_pos.z,		\
					   &ins_ltp_speed.x,	\
					   &ins_ltp_speed.y,	\
					   &ins_ltp_speed.z,	\
					   &ins_ltp_accel.x,	\
					   &ins_ltp_accel.y,	\
					   &ins_ltp_accel.z);	\
  }

#define PERIODIC_SEND_INS_REF(_chan) {				\
    DOWNLINK_SEND_INS_REF(_chan,					\
				&ins_ltp_def.ecef.x,		\
				&ins_ltp_def.ecef.y,		\
				&ins_ltp_def.ecef.z,		\
				&ins_ltp_def.lla.lat,		\
				&ins_ltp_def.lla.lon,		\
				&ins_ltp_def.lla.alt,		\
				&ins_ltp_def.hmsl,		\
				&ins_qfe);				\
  }

#define PERIODIC_SEND_VERT_LOOP(_chan) {				\
    DOWNLINK_SEND_VERT_LOOP(_chan,				\
				  &guidance_v_z_sp,		\
				  &guidance_v_zd_sp,		\
				  &ins_ltp_pos.z,			\
				  &ins_ltp_speed.z,		\
				  &ins_ltp_accel.z,		\
				  &guidance_v_z_ref,		\
				  &guidance_v_zd_ref,		\
				  &guidance_v_zdd_ref,		\
				  &gv_adapt_X,			\
				  &gv_adapt_P,			\
				  &gv_adapt_Xmeas,			\
				  &guidance_v_z_sum_err,		\
				  &guidance_v_ff_cmd,		\
				  &guidance_v_fb_cmd,		\
				  &guidance_v_delta_t);		\
  }

#define PERIODIC_SEND_HOVER_LOOP(_chan) {				\
    DOWNLINK_SEND_HOVER_LOOP(_chan,				\
				   &guidance_h_pos_sp.x,		\
				   &guidance_h_pos_sp.y,		\
				   &ins_ltp_pos.x,			\
				   &ins_ltp_pos.y,			\
				   &ins_ltp_speed.x,		\
				   &ins_ltp_speed.y,		\
				   &ins_ltp_accel.x,		\
				   &ins_ltp_accel.y,		\
				   &guidance_h_pos_err.x,		\
				   &guidance_h_pos_err.y,		\
				   &guidance_h_speed_err.x,	\
				   &guidance_h_speed_err.y,	\
				   &guidance_h_pos_err_sum.x,	\
				   &guidance_h_pos_err_sum.y,	\
				   &guidance_h_nav_err.x,	\
				   &guidance_h_nav_err.y,	\
				   &guidance_h_command_earth.x,	\
				   &guidance_h_command_earth.y,	\
				   &guidance_h_command_body.phi,	\
				   &guidance_h_command_body.theta, \
				   &guidance_h_command_body.psi);	\
  }

#define PERIODIC_SEND_GUIDANCE_H_REF(_chan) { \
  DOWNLINK_SEND_GUIDANCE_H_REF_INT(_chan, \
      &guidance_h_pos_sp.x, \
      &guidance_h_pos_ref.x, \
      &guidance_h_speed_ref.x, \
      &guidance_h_accel_ref.x, \
      &guidance_h_pos_sp.y, \
      &guidance_h_pos_ref.y, \
      &guidance_h_speed_ref.y, \
      &guidance_h_accel_ref.y); \
}

#include "booz_gps.h"
#include "firmwares/rotorcraft/navigation.h"
#define PERIODIC_SEND_BOOZ2_FP(_chan) {					\
    int32_t carrot_up = -guidance_v_z_sp;				\
    DOWNLINK_SEND_BOOZ2_FP( _chan,					\
			    &ins_enu_pos.x,			\
			    &ins_enu_pos.y,			\
			    &ins_enu_pos.z,			\
			    &ins_enu_speed.x,			\
			    &ins_enu_speed.y,			\
			    &ins_enu_speed.z,			\
			    &ahrs.ltp_to_body_euler.phi,		\
			    &ahrs.ltp_to_body_euler.theta,		\
			    &ahrs.ltp_to_body_euler.psi,		\
			    &guidance_h_pos_sp.y,			\
			    &guidance_h_pos_sp.x,			\
			    &carrot_up,					\
			    &guidance_h_command_body.psi,		\
			    &stabilization_cmd[COMMAND_THRUST], \
          &autopilot_flight_time);	\
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
			     &booz_gps_state.tow,			\
			     &booz_gps_state.pdop,			\
			     &booz_gps_state.num_sv,			\
			     &booz_gps_state.fix);			\
  }
#else
#define PERIODIC_SEND_BOOZ2_GPS(_chan) {}
#endif

#include "firmwares/rotorcraft/navigation.h"
#define PERIODIC_SEND_ROTORCRAFT_NAV_STATUS(_chan) {				\
    DOWNLINK_SEND_ROTORCRAFT_NAV_STATUS(_chan,                      \
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
#define PERIODIC_SEND_BOOZ2_CAM(_chan) DOWNLINK_SEND_BOOZ2_CAM(_chan,&booz_cam_tilt,&booz_cam_pan);
#else
#define PERIODIC_SEND_BOOZ2_CAM(_chan) {}
#endif

#define PERIODIC_SEND_BOOZ2_TUNE_HOVER(_chan) {				       \
    DOWNLINK_SEND_BOOZ2_TUNE_HOVER(_chan,				       \
				   &radio_control.values[RADIO_ROLL],  \
				   &radio_control.values[RADIO_PITCH], \
				   &radio_control.values[RADIO_YAW],   \
				   &stabilization_cmd[COMMAND_ROLL],      \
				   &stabilization_cmd[COMMAND_PITCH],     \
				   &stabilization_cmd[COMMAND_YAW],       \
				   &stabilization_cmd[COMMAND_THRUST],    \
				   &ahrs.ltp_to_imu_euler.phi,	       \
				   &ahrs.ltp_to_imu_euler.theta,	       \
				   &ahrs.ltp_to_imu_euler.psi,	       \
				   &ahrs.ltp_to_body_euler.phi,	       \
				   &ahrs.ltp_to_body_euler.theta,	       \
				   &ahrs.ltp_to_body_euler.psi	       \
				   );					       \
  }

#define PERIODIC_SEND_I2C_ERRORS(_chan) {				       \
    DOWNLINK_SEND_I2C_ERRORS(_chan,				       \
				   &i2c_errc_ack_fail,  \
				   &i2c_errc_miss_start_stop,  \
				   &i2c_errc_arb_lost,  \
				   &i2c_errc_over_under,  \
				   &i2c_errc_pec_recep,  \
				   &i2c_errc_timeout_tlow,  \
				   &i2c_errc_smbus_alert  \
				   );					       \
  }

//TODO replace by BOOZ_EXTRA_ADC
#ifdef BOOZ2_SONAR
#define PERIODIC_SEND_BOOZ2_SONAR(_chan) DOWNLINK_SEND_BOOZ2_SONAR(_chan,&booz2_adc_1,&booz2_adc_2,&booz2_adc_3,&booz2_adc_4);
#else
#define PERIODIC_SEND_BOOZ2_SONAR(_chan) {}
#endif

#ifdef BOOZ2_TRACK_CAM
#include "cam_track.h"
#define PERIODIC_SEND_CAM_TRACK(_chan) DOWNLINK_SEND_BOOZ_SIM_SPEED_POS(_chan, \
    &target_accel_ned.x, \
    &target_accel_ned.y, \
    &target_accel_ned.z, \
    &target_speed_ned.x, \
    &target_speed_ned.y, \
    &target_speed_ned.z, \
    &target_pos_ned.x, \
    &target_pos_ned.y, \
    &target_pos_ned.z)
#else
#define PERIODIC_SEND_CAM_TRACK(_chan) {}
#endif

#include "generated/settings.h"
#define PERIODIC_SEND_DL_VALUE(_chan) PeriodicSendDlValue(_chan)

#include "generated/periodic.h"
#define Booz2TelemetryPeriodic() {			\
    PeriodicSendMain(DefaultChannel);			\
  }


#endif /* TELEMETRY_H */
