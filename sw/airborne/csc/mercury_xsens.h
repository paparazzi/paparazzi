/*
 * Copyright (C) 2008 Joby Energy
 *  
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

/** \file xsens.h
 */

#ifndef __MERCURY_XSENS_H__
#define __MERCURY_XSENS_H__

#include "types.h"

#define XSENS_COUNT 1

extern uint8_t xsens_mode[XSENS_COUNT]; // Receiver status 
extern volatile uint8_t xsens_msg_received[XSENS_COUNT];

extern float xsens_phi[XSENS_COUNT];
extern float xsens_theta[XSENS_COUNT];
extern float xsens_psi[XSENS_COUNT];
extern uint8_t xsens_msg_status[XSENS_COUNT];

extern float xsens_r_a[XSENS_COUNT];
extern float xsens_r_b[XSENS_COUNT];
extern float xsens_r_c[XSENS_COUNT];
extern float xsens_r_d[XSENS_COUNT];
extern float xsens_r_e[XSENS_COUNT];
extern float xsens_r_f[XSENS_COUNT];
extern float xsens_r_g[XSENS_COUNT];
extern float xsens_r_h[XSENS_COUNT];
extern float xsens_r_i[XSENS_COUNT];

extern float xsens_accel_x[XSENS_COUNT];
extern float xsens_accel_y[XSENS_COUNT];
extern float xsens_accel_z[XSENS_COUNT];
extern float xsens_mag_x[XSENS_COUNT];
extern float xsens_mag_y[XSENS_COUNT];
extern float xsens_mag_z[XSENS_COUNT];
extern float xsens_gyro_x[XSENS_COUNT];
extern float xsens_gyro_y[XSENS_COUNT];
extern float xsens_gyro_z[XSENS_COUNT];
extern float xsens_mag_heading[XSENS_COUNT];

extern uint16_t xsens_time_stamp[XSENS_COUNT];

extern int xsens_setzero;

#include "booz_ahrs.h"

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

#define PERIODIC_SEND_BOOZ2_AHRS_EULER() {			\
    DOWNLINK_SEND_BOOZ2_AHRS_EULER(&booz_ahrs.ltp_to_imu_euler.phi,	\
				   &booz_ahrs.ltp_to_imu_euler.theta,	\
				   &booz_ahrs.ltp_to_imu_euler.psi,	\
				   &booz_ahrs.ltp_to_body_euler.phi,	\
				   &booz_ahrs.ltp_to_body_euler.theta,	\
				   &booz_ahrs.ltp_to_body_euler.psi);	\
  }



#include "std.h"

void xsens_init(void);
void xsens_parse_msg(uint8_t xsens_id);
void xsens_event_task(void);
void xsens_periodic_task(void);

#endif 
