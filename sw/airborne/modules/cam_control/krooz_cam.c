/*
 * $Id: $
 *
 * Copyright (C) 2012 Sergey Krukowski <softsr@yahoo.de>
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

#include "subsystems/actuators/actuators_pwm.h"
#include "cam_control/krooz_cam.h"
#include "subsystems/ahrs.h"
#include "subsystems/ins.h"
#include "generated/flight_plan.h"
#include "subsystems/radio_control.h"
#include "subsystems/navigation/common_nav.h"

#ifndef TILT_COEFF
#define TILT_COEFF 500
#endif
#ifndef PAN_COEFF
#define PAN_COEFF 500
#endif
#ifndef TILT_RATE
#define TILT_RATE 50
#endif
#ifndef PAN_RATE
#define PAN_RATE 50
#endif
#ifndef PAN_CENTER
#define PAN_CENTER 1500
#endif
#ifndef TILT_CENTER
#define TILT_CENTER 1500
#endif
#ifndef PAN_AREA
#define PAN_AREA 300
#endif
#ifndef PAN_DOWN
#define PAN_DOWN 1800
#endif

bool_t servo_flag;
#ifdef CAM_SETUP
int16_t tilt_center = TILT_CENTER;
int16_t tilt_coeff = TILT_COEFF;
int16_t pan_coeff = PAN_COEFF;
int16_t tilt_rate = TILT_RATE;
int16_t pan_rate = PAN_RATE;
int16_t pan_down = PAN_DOWN;
#endif
int16_t cam_tilt = 0;
int16_t cam_pan = 0;
int32_t radio_cam_value = 0;

void krooz_cam_periodic() {
	servo_flag = TRUE;
}

void krooz_cam_event(void) {
	if(servo_flag) {
		//if(!stage_complete || autopilot_mode != AP_MODE_NAV) 
		{
#ifdef CAM_SETUP
			int32_t angle_buf = ahrs.ltp_to_body_euler.phi*tilt_coeff/INT32_ANGLE_PI;
			int32_t rate_buf = ahrs.body_rate.p*tilt_rate/INT32_ANGLE_PI;
			cam_tilt = (int16_t)angle_buf + (int16_t)rate_buf + tilt_center;
			angle_buf = ahrs.ltp_to_body_euler.theta*pan_coeff/INT32_ANGLE_PI;
			rate_buf = ahrs.body_rate.q*pan_rate/INT32_ANGLE_PI;
#else
			int32_t angle_buf = ahrs.ltp_to_body_euler.phi*TILT_COEFF/INT32_ANGLE_PI;
			int32_t rate_buf = ahrs.body_rate.p*TILT_RATE/INT32_ANGLE_PI;
			cam_tilt = (int16_t)angle_buf + (int16_t)rate_buf + TILT_CENTER;
			angle_buf = ahrs.ltp_to_body_euler.theta*PAN_COEFF/INT32_ANGLE_PI;
			rate_buf = ahrs.body_rate.q*PAN_RATE/INT32_ANGLE_PI;
#endif
			if(radio_control.status == RC_OK)
				radio_cam_value = (radio_cam_value*49 + (int32_t)radio_control.values[RADIO_CAM]) / 50;
			cam_pan = (int16_t)angle_buf + radio_cam_value*PAN_AREA/MAX_PPRZ + (int16_t)rate_buf + PAN_CENTER;
		#ifdef CAM_TEST
			cam_pan = (int16_t)angle_buf + radio_cam_value*600/MAX_PPRZ + (int16_t)rate_buf + PAN_CENTER;
		#endif
		}
/*		else {
			cam_tilt = TILT_CENTER;
			cam_pan = PAN_DOWN;
		}*/
		Bound(cam_tilt, 900, 2100);
		Bound(cam_pan, 900, 2100);
	#ifndef SITL
		actuators_pwm_values[TILT] = SERVOS_TICS_OF_USEC(cam_tilt);
		actuators_pwm_values[PAN] = SERVOS_TICS_OF_USEC(cam_pan);
	#endif
	}
	servo_flag = FALSE;
}

