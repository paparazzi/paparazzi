/*
 * Copyright (C) 2010 ENAC
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

/**
 * \brief Library for the XSENS AHRS
 */

#ifndef INS_XSENS_H
#define INS_XSENS_H

#include "std.h"

#include "ins_module.h"

struct XsensTime {
  int8_t hour;
  int8_t min;
  int8_t sec;
  int32_t nanosec;
  int16_t year;
  int8_t month;
  int8_t day;
};

extern struct XsensTime xsens_time;

extern uint8_t xsens_msg_status;
extern uint16_t xsens_time_stamp;

extern void xsens_periodic(void);

/* To use Xsens to just provide IMU measurements
 * for use with an external AHRS algorithm
 */
#if USE_IMU
#include "subsystems/imu.h"

struct ImuXsens {
  bool_t gyro_available;
  bool_t accel_available;
  bool_t mag_available;
};
extern struct ImuXsens imu_xsens;

#define ImuEvent(_gyro_handler, _accel_handler, _mag_handler) { \
    if (imu_xsens.accel_available) {                            \
      imu_xsens.accel_available = FALSE;                        \
      _accel_handler();                                         \
    }                                                           \
    if (imu_xsens.gyro_available) {                             \
      imu_xsens.gyro_available = FALSE;                         \
      _gyro_handler();                                          \
    }                                                           \
    if (imu_xsens.mag_available) {                              \
      imu_xsens.mag_available = FALSE;                          \
      _mag_handler();                                           \
    }                                                           \
  }
#endif /* USE_IMU */


/* use Xsens as a full INS solution */
#if USE_INS_MODULE
#define InsEvent(_ins_handler) {  \
    ins_event_check_and_handle(handle_ins_msg);   \
  }
#define DefaultInsImpl ins_xsens
#define InsPeriodic xsens_periodic
extern void ins_xsens_init(void);
extern void ins_xsens_register(void);
#endif


#if USE_GPS_XSENS
extern bool_t gps_xsens_msg_available;
#define GpsEvent(_sol_available_callback) {         \
    if (gps_xsens_msg_available) {                  \
      gps.last_msg_ticks = sys_time.nb_sec_rem;     \
      gps.last_msg_time = sys_time.nb_sec;          \
      if (gps.fix == GPS_FIX_3D) {                  \
        gps.last_3dfix_ticks = sys_time.nb_sec_rem; \
        gps.last_3dfix_time = sys_time.nb_sec;      \
      }                                             \
      _sol_available_callback();                    \
      gps_xsens_msg_available = FALSE;              \
    }                                               \
  }
#endif

#endif
