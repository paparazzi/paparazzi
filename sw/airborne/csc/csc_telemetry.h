/*
 * $Id: booz2_telemetry.c 3002 2009-02-10 11:36:07Z poine $
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

#ifndef CSC_TELEMETRY_H
#define CSC_TELEMETRY_H

#include "actuators.h"
#include <inttypes.h>

extern uint8_t telemetry_mode_Ap_DefaultChannel;

#include "downlink.h"
#include "settings.h"
#include "booz/booz2_gps.h"


#define PERIODIC_SEND_DL_VALUE(_chan) PeriodicSendDlValue(_chan)

#define PERIODIC_SEND_ALIVE(_chan) DOWNLINK_SEND_ALIVE(_chan, 16, MD5SUM)

#define PERIODIC_SEND_DOWNLINK(_chan) { \
  static uint16_t last; \
  uint16_t rate = (downlink_nb_bytes - last) / PERIOD_DOWNLINK_0; \
  last = downlink_nb_bytes; \
  DOWNLINK_SEND_DOWNLINK(_chan, &downlink_nb_ovrn, &rate, &downlink_nb_msgs); \
}


#ifdef PROPS_NB

#include "mercury_ap.h"
#define PERIODIC_SEND_MERCURY_PROPS(_chan) DOWNLINK_SEND_MERCURY_PROPS(_chan, &(mixed_commands[0]),&(mixed_commands[1]),&(mixed_commands[2]),&(mixed_commands[3]))

#endif /*  PROPS_NB */

#ifdef BUSS_TWI_BLMC_NB
#define PERIODIC_SEND_MERCURY_PROPS(_chan) DOWNLINK_SEND_MERCURY_PROPS(_chan, &(motor_power[0]),&(motor_power[1]),&(motor_power[2]),&(motor_power[3]))
#endif /* BUSS_TWI_BLMC_NB */

#ifdef COMMANDS_NB
#include "commands.h"
#define PERIODIC_SEND_COMMANDS(_chan) DOWNLINK_SEND_COMMANDS(_chan, COMMANDS_NB, commands)
#endif /* COMMANDS_NB */
#define PERIODIC_SEND_SIMPLE_COMMANDS(_chan) DOWNLINK_SEND_SIMPLE_COMMANDS(_chan, &commands[0], &commands[1], &commands[2])
#define PERIODIC_SEND_CONTROLLER_GAINS(_chan) DOWNLINK_SEND_CONTROLLER_GAINS(_chan, &csc_gains.roll_kp, &csc_gains.roll_kd, &csc_gains.roll_ki, &csc_gains.pitch_kp, &csc_gains.pitch_kd, &csc_gains.pitch_ki, &csc_gains.yaw_kp, &csc_gains.yaw_kd, &csc_gains.yaw_ki)

#ifdef SERVOS_NB
#define PERIODIC_SEND_ACTUATORS(_chan) DOWNLINK_SEND_ACTUATORS(_chan, SERVOS_NB, actuators)
#endif

#ifdef USE_RADIO_CONTROL
#include "booz_radio_control.h"
#define PERIODIC_SEND_RC(_chan) DOWNLINK_SEND_RC(_chan, RADIO_CONTROL_NB_CHANNEL, radio_control.values)
#define PERIODIC_SEND_FBW_STATUS(_chan) { uint16_t current; DOWNLINK_SEND_FBW_STATUS(_chan, &radio_control.status, &pprz_mode, &vsupply, &current); }
#else 
#ifdef RADIO_CONTROL
#define PERIODIC_SEND_RC(_chan) DOWNLINK_SEND_RC(_chan, PPM_NB_PULSES, rc_values)
#define PERIODIC_SEND_FBW_STATUS(_chan) { uint16_t current; DOWNLINK_SEND_FBW_STATUS(_chan, &rc_status, &pprz_mode, &vsupply, &current); }
#endif
#endif /* USE_RADIO_CONTROL */

#ifdef USE_GPS
#define PERIODIC_SEND_BOOZ2_GPS(_chan) {			\
DOWNLINK_SEND_BOOZ2_GPS( _chan,    				\
                         &booz_ins_gps_pos_cm_ned.x,		\
                         &booz_ins_gps_pos_cm_ned.y,		\
			     &booz_ins_gps_pos_cm_ned.z,	\
			     &booz_gps_state.ecef_speed.x,	\
			     &booz_gps_state.ecef_speed.y,	\
			     &booz_gps_state.ecef_speed.z,	\
			     &booz_gps_state.pacc,		\
			     &booz_gps_state.sacc,		\
			     &booz_gps_state.pdop,		\
			     &booz_gps_state.num_sv,		\
			     &booz_gps_state.fix)		\
  }
#endif

#define PERIODIC_SEND_GPS_ERROR(_chan) {				\
DOWNLINK_SEND_GPS_ERROR( _chan, 				\
			     &csc_gps_errors.pos.x,		\
			   &csc_gps_errors.pos.y,		\
			   &csc_gps_errors.pos.z,		\
			   &csc_gps_errors.rate.x,		\
			   &csc_gps_errors.rate.y,		\
			   &csc_gps_errors.rate.z) 		\
    }			   
	
#define PERIODIC_SEND_BOOZ2_INS3(_chan) { \
DOWNLINK_SEND_BOOZ2_INS3(_chan,	  \
&booz_ins_gps_pos_cm_ned.x,	\
&booz_ins_gps_pos_cm_ned.y,	    \
&booz_ins_gps_pos_cm_ned.z,	    \
&booz_ins_gps_speed_cm_s_ned.x,   \
&booz_ins_gps_speed_cm_s_ned.y,   \
&booz_ins_gps_speed_cm_s_ned.z    \
) }

#define PERIODIC_SEND_GPS_SOL(_chan) DOWNLINK_SEND_GPS_SOL(_chan, &booz_gps_state.pacc, \
  &booz_gps_state.sacc, &booz_gps_state.pdop, &booz_gps_state.num_sv)

#define PERIODIC_SEND_GPS(_chan) { uint32_t zero = 0; DOWNLINK_SEND_GPS(_chan, &booz_gps_state.fix, \
  &booz_ins_gps_pos_cm_ned.y, \
  &booz_ins_gps_pos_cm_ned.x, \
  &zero, \
  &booz_ins_gps_pos_cm_ned.z, \
  &zero, \
  &zero, \
  &zero, \
  &zero, \
  &zero, &zero) }

#ifdef USE_AIRSPEED
#include "estimator.h"
#define PERIODIC_SEND_AIRSPEED(_chan) { float empty; DOWNLINK_SEND_AIRSPEED (_chan, &adc_airspeed_val,&estimator_airspeed,&empty,&empty,&empty,&empty) }
#else
#define PERIODIC_SEND_AIRSPEED(_chan) {}
#endif

#ifdef USE_BARO_ETS
#include "baro_ets.h"
#define PERIODIC_SEND_ESTIMATOR(_chan) { float empty; DOWNLINK_SEND_ESTIMATOR(_chan, &baro_ets_altitude, &empty) }
#endif

#endif /* CSC_TELEMETRY_H */
