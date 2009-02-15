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
    DOWNLINK_SEND_BOOZ2_GYRO(&booz2_imu_gyro.x,		\
			     &booz2_imu_gyro.y,		\
			     &booz2_imu_gyro.z);	\
  }

#define PERIODIC_SEND_BOOZ2_ACCEL() {				\
    DOWNLINK_SEND_BOOZ2_ACCEL(&booz2_imu_accel.x,		\
			      &booz2_imu_accel.y,		\
			      &booz2_imu_accel.z);		\
  }

#define PERIODIC_SEND_BOOZ2_MAG() {				\
    DOWNLINK_SEND_BOOZ2_MAG(&booz2_imu_mag.x,			\
			    &booz2_imu_mag.y,			\
			    &booz2_imu_mag.z);			\
  }



#define PERIODIC_SEND_IMU_GYRO_RAW() {					\
    DOWNLINK_SEND_IMU_GYRO_RAW(&booz2_imu_gyro_unscaled.x,		\
			       &booz2_imu_gyro_unscaled.y,		\
			       &booz2_imu_gyro_unscaled.z);		\
  }

#define PERIODIC_SEND_IMU_ACCEL_RAW() {					\
    DOWNLINK_SEND_IMU_ACCEL_RAW(&booz2_imu_accel_unscaled.x,		\
				&booz2_imu_accel_unscaled.y,		\
				&booz2_imu_accel_unscaled.z);		\
  }

#define PERIODIC_SEND_IMU_MAG_RAW() {					\
    DOWNLINK_SEND_IMU_MAG_RAW(&booz2_imu_mag_unscaled.x,		\
			      &booz2_imu_mag_unscaled.y,		\
			      &booz2_imu_mag_unscaled.z);		\
  }




#include "booz2_imu.h"
#include "booz2_stabilization.h"
#include "booz2_stabilization_rate.h"
#define PERIODIC_SEND_BOOZ2_RATE_LOOP() {				\
    DOWNLINK_SEND_BOOZ2_RATE_LOOP(&booz2_stabilization_rate_measure.x,	\
				  &booz2_stabilization_rate_measure.y,	\
				  &booz2_stabilization_rate_measure.z,	\
				  &booz2_stabilization_rate_sp.x,	\
				  &booz2_stabilization_rate_sp.y,	\
				  &booz2_stabilization_rate_sp.z,	\
				  &booz2_stabilization_cmd[COMMAND_ROLL], \
				  &booz2_stabilization_cmd[COMMAND_PITCH], \
				  &booz2_stabilization_cmd[COMMAND_YAW], \
				  &booz2_stabilization_cmd[COMMAND_THRUST]); \
  }


#include "booz2_stabilization_attitude.h"
#define PERIODIC_SEND_BOOZ2_STAB_ATTITUDE() {				\
    DOWNLINK_SEND_BOOZ2_STAB_ATTITUDE(&booz2_filter_attitude_rate.x,	\
				      &booz2_filter_attitude_rate.y,	\
				      &booz2_filter_attitude_rate.z,	\
				      &booz2_filter_attitude_euler_aligned.phi,	\
				      &booz2_filter_attitude_euler_aligned.theta, \
				      &booz2_filter_attitude_euler_aligned.psi,	\
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
					      &booz2_filter_attitude_euler_aligned.psi,	\
					      &booz2_filter_attitude_rate.z, \
					      &booz_stabilization_att_sum_err.psi, \
					      &booz2_stabilization_cmd[COMMAND_YAW]); \
  }
#else
#define PERIODIC_SEND_BOOZ2_STAB_ATTITUDE_HS_ROLL() {			       \
    DOWNLINK_SEND_BOOZ2_STAB_ATTITUDE_HS_ROLL(&booz_stabilization_att_ref.phi, \
					      &booz_stabilization_rate_ref.x,  \
					      &booz_stabilization_accel_ref.x, \
					      &booz2_filter_attitude_euler_aligned.phi,	\
					      &booz2_filter_attitude_rate.x, \
					      &booz_stabilization_att_sum_err.phi, \
					      &booz2_stabilization_cmd[COMMAND_ROLL]); \
  }
#endif


#include "booz2_filter_aligner.h"
#define PERIODIC_SEND_BOOZ2_FILTER_ALIGNER() {				\
    DOWNLINK_SEND_BOOZ2_FILTER_ALIGNER(&booz2_filter_aligner_lp_gyro.x,	\
				       &booz2_filter_aligner_lp_gyro.y,	\
				       &booz2_filter_aligner_lp_gyro.z,	\
				       &booz2_imu_gyro.x,		\
				       &booz2_imu_gyro.y,		\
				       &booz2_imu_gyro.z,		\
				       &booz2_filter_aligner_noise,	\
				       &booz2_filter_aligner_low_noise_cnt); \
  }


#define PERIODIC_SEND_BOOZ2_CMD() { \
    DOWNLINK_SEND_BOOZ2_CMD(&booz2_stabilization_cmd[COMMAND_ROLL],	\
			    &booz2_stabilization_cmd[COMMAND_PITCH],	\
			    &booz2_stabilization_cmd[COMMAND_YAW],	\
			    &booz2_stabilization_cmd[COMMAND_THRUST]);	\
  }



#include "booz2_filter_attitude_cmpl_euler.h"
#define PERIODIC_SEND_BOOZ2_FILTER() {					\
    DOWNLINK_SEND_BOOZ2_FILTER(&booz2_filter_attitude_euler.phi,	\
			       &booz2_filter_attitude_euler.theta,	\
			       &booz2_filter_attitude_euler.psi,	\
			       &booz2_face_measure.phi,			\
			       &booz2_face_measure.theta,		\
			       &booz2_face_measure.psi,			\
			       &booz2_face_corrected.phi,		\
			       &booz2_face_corrected.theta,		\
			       &booz2_face_corrected.psi,		\
			       &booz2_face_residual.phi,		\
			       &booz2_face_residual.theta,		\
			       &booz2_face_residual.psi,		\
			       &booz2_face_gyro_bias.x,			\
			       &booz2_face_gyro_bias.y,			\
			       &booz2_face_gyro_bias.z);		\
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
			    &booz_ins_position.z,		\
			    &booz_ins_speed_earth.z,		\
			    &booz_ins_accel_earth.z);		\
  }
#include "booz2_guidance_v.h"
#define PERIODIC_SEND_BOOZ2_VERT_LOOP() {				\
    DOWNLINK_SEND_BOOZ2_VERT_LOOP(&booz2_guidance_v_z_sp,		\
				  &booz2_guidance_v_zd_sp,		\
				  &booz_ins_position.z,			\
				  &booz_ins_speed_earth.z,		\
				  &booz_ins_accel_earth.z,		\
				  &booz2_guidance_v_z_ref,		\
				  &booz2_guidance_v_zd_ref,		\
				  &booz2_guidance_v_zdd_ref,		\
				  &b2_gv_adapt_X,			\
				  &b2_gv_adapt_P,			\
				  &booz2_guidance_v_z_sum_err,		\
				  &booz2_guidance_v_ff_cmd,		\
				  &booz2_guidance_v_fb_cmd,		\
				  &booz2_guidance_v_delta_t);		\
  }

#define PERIODIC_SEND_BOOZ2_HOVER_LOOP() {				\
    DOWNLINK_SEND_BOOZ2_HOVER_LOOP(&booz_ins_position.x,	\
				   &booz_ins_position.y,	\
				   &booz2_guidance_h_pos_sp.x,		\
				   &booz2_guidance_h_pos_sp.y,		\
				   &booz_ins_speed_earth.x,	\
				   &booz_ins_speed_earth.y,	\
				   &booz2_guidance_h_pos_err.x,		\
				   &booz2_guidance_h_pos_err.y,		\
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
    DOWNLINK_SEND_BOOZ2_FP( &booz_ins_position_lla.lon,				\
			    &booz_ins_position_lla.lat,				\
			    &booz_ins_position.z,				\
			    &booz_ins_speed_earth.x,				\
			    &booz_ins_speed_earth.y,				\
			    &booz2_filter_attitude_euler_aligned.phi,	\
			    &booz2_filter_attitude_euler_aligned.theta,	\
			    &booz2_filter_attitude_euler_aligned.psi,	\
			    &booz2_guidance_h_pos_sp.y,		\
			    &booz2_guidance_h_pos_sp.x,		\
			    &booz2_guidance_v_z_sp,			\
			    &booz2_guidance_h_command_body.psi, \
			    &booz2_stabilization_cmd[COMMAND_THRUST]);	\
  }


#define PERIODIC_SEND_BOOZ2_NAV_REF() { \
  DOWNLINK_SEND_BOOZ2_NAV_REF(					\
  &booz_ins_position_init_lla.lon,		\
  &booz_ins_position_init_lla.lat)		\
  }


#define PERIODIC_SEND_BOOZ2_TUNE_HOVER() {				\
    DOWNLINK_SEND_BOOZ2_TUNE_HOVER(&rc_values[RADIO_ROLL],		\
				   &rc_values[RADIO_PITCH],		\
				   &rc_values[RADIO_YAW],		\
				   &booz2_stabilization_cmd[COMMAND_ROLL], \
				   &booz2_stabilization_cmd[COMMAND_PITCH], \
				   &booz2_stabilization_cmd[COMMAND_YAW], \
				   &booz2_stabilization_cmd[COMMAND_THRUST], \
				   &booz2_filter_attitude_euler.phi,	\
				   &booz2_filter_attitude_euler.theta,	\
				   &booz2_filter_attitude_euler.psi,	\
				   &booz2_filter_attitude_euler_aligned.phi, \
				   &booz2_filter_attitude_euler_aligned.theta, \
				   &booz2_filter_attitude_euler_aligned.psi \
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
