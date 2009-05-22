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

#ifndef __CSC_XSENS_H__
#define __CSC_XSENS_H__

#include "types.h"

#define XSENS_COUNT 1

extern uint8_t xsens_mode[XSENS_COUNT]; // Receiver status 
extern volatile uint8_t xsens_msg_received[XSENS_COUNT];

extern float xsens_pitch[XSENS_COUNT];
extern float xsens_roll[XSENS_COUNT];
extern float xsens_yaw[XSENS_COUNT];
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

#define PERIODIC_SEND_IMU_GYRO() DOWNLINK_SEND_IMU_GYRO (\
  &xsens_gyro_x, &xsens_gyro_y, &xsens_gyro_z \
)

#define PERIODIC_SEND_IMU_ACCEL() DOWNLINK_SEND_IMU_ACCEL (\
  &xsens_accel_x, &xsens_accel_y, &xsens_accel_z \
)

#define PERIODIC_SEND_IMU_MAG() DOWNLINK_SEND_IMU_MAG (\
  &xsens_mag_x, &xsens_mag_y, &xsens_mag_z \
)

/*
#define PERIODIC_SEND_XSENS_DATA() DOWNLINK_SEND_XSENS_DATA (\
  &whirly_timestamp, \
  xsens_accel_x, xsens_accel_y, xsens_accel_z, \
  xsens_mag_x, xsens_mag_y, xsens_mag_z, \
  xsens_gyro_x, xsens_gyro_y, xsens_gyro_z, \
  xsens_time_stamp, \
  xsens_accel_x + 1, xsens_accel_y + 1, xsens_accel_z + 1, \
  xsens_mag_x + 1, xsens_mag_y + 1, xsens_mag_z + 1, \
  xsens_gyro_x + 1, xsens_gyro_y + 1, xsens_gyro_z + 1, \
  xsens_time_stamp + 1 \
)
*/

#include "std.h"

void xsens_init(void);
void xsens_parse_msg(uint8_t xsens_id);
void xsens_event_task(void);
void xsens_periodic_task(void);

#endif 
