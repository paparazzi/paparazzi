/*
 * Copyright (C) 2015 Michal Podhradsky, michal.podhradsky@aggiemail.usu.edu
 * Utah State University, http://aggieair.usu.edu/
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
/**
 * @file ins_vectornav.h
 *
 * Vectornav VN-200 INS subsystem
 *
 * @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 */
#ifndef INS_VECTORNAV_H
#define INS_VECTORNAV_H

// Subsystems
#include "subsystems/imu.h"
#include "subsystems/gps.h"
#include "subsystems/ahrs.h"
#include "subsystems/ins.h"

// Peripheral
#include "peripherals/vn200_serial.h"

// Geodetic / Math
#include "math/pprz_algebra.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_isa.h"

// Generated
#include "generated/airframe.h"
#include "generated/flight_plan.h"

// Systime
#include "mcu_periph/sys_time.h"

// Abi
#include "subsystems/abi.h"

#if GPS_USE_LATLONG
/* currently needed to get nav_utm_zone0 */
#include "subsystems/navigation/common_nav.h"
#include "math/pprz_geodetic_float.h"
#endif

/*
 * IMU Subsystem defines
 */
extern void imu_vectornav_event(void);

#if !USE_NPS
#define ImuEvent imu_vectornav_event
#endif /* Only for non-NPS targets */

/*
 * IMU scalling defaults - still needed, although not used
 * Filled in with dummy values (ones)
 */
#if !defined IMU_GYRO_P_SENS & !defined IMU_GYRO_Q_SENS & !defined IMU_GYRO_R_SENS
#define IMU_GYRO_P_SENS 1
#define IMU_GYRO_P_SENS_NUM 1
#define IMU_GYRO_P_SENS_DEN 1
#define IMU_GYRO_Q_SENS 1
#define IMU_GYRO_Q_SENS_NUM 1
#define IMU_GYRO_Q_SENS_DEN 1
#define IMU_GYRO_R_SENS 1
#define IMU_GYRO_R_SENS_NUM 1
#define IMU_GYRO_R_SENS_DEN 1
#endif

#if !defined IMU_ACCEL_X_SENS & !defined IMU_ACCEL_Y_SENS & !defined IMU_ACCEL_Z_SENS
#define IMU_ACCEL_X_SENS 1
#define IMU_ACCEL_X_SENS_NUM 1
#define IMU_ACCEL_X_SENS_DEN 1
#define IMU_ACCEL_Y_SENS 1
#define IMU_ACCEL_Y_SENS_NUM 1
#define IMU_ACCEL_Y_SENS_DEN 1
#define IMU_ACCEL_Z_SENS 1
#define IMU_ACCEL_Z_SENS_NUM 1
#define IMU_ACCEL_Z_SENS_DEN 1
#endif


/*
 * GPS subsystem defines
 */
#if !USE_NPS
#define GpsEvent(_x) ;
#endif /* Only for non-NPS targets */

/*
 * AHRS subsystem
 * - uses ins_vectonav_wrapper.h
 */

/*
 * INS subsystem defines
 */
#ifndef DefaultInsImpl
#define DefaultInsImpl ins_vectornav
#endif




// Ins implementation state (fixed point)
struct InsVectornav {
  struct LtpDef_i  ltp_def; // initial position
  bool_t           ltp_initialized; // status indicator

  // output LTP NED for telemetry messages
  struct NedCoor_i ltp_pos;
  struct NedCoor_i ltp_speed;
  struct NedCoor_i ltp_accel;

  struct LlaCoor_f lla_pos;

  struct NedCoor_f ltp_accel_f;

  // baro [height above ground]
  float baro_z;  ///< z-position calculated from baro in meters (z-down)
  float qfe;

  // Packet data
  struct VNPacket vn_packet;///< Packet struct
  enum VNStatus vn_status;  ///< VN status
  float vn_freq;            ///< data frequency
  uint16_t vn_chksm;         ///< aux variable for checksum
  uint32_t vn_time;          ///< VN time stamp
  uint32_t vn_ltime;         ///< aux time stamp

  // Auxilliary data fields
  float timestamp; // System time [s]
  struct FloatEulers attitude; // Attitude, float, [degrees], yaw, pitch, roll
  // rates -> imu
  double pos_lla[3]; // Lla [deg, deg, m above elipsoid]
  struct NedCoor_f vel_ned; // The estimated velocity in the North East Down (NED) frame, given in m/s.
  // accel -> imu
  // num sats -> GPS
  // GPS fix -> GPS
  float posU[3]; // The current GPS position uncertainty in the North East Down (NED) coordinate frame, given in meters.
  float velU; // NED velocity uncertainty [m/s]
  struct FloatVect3 lin_accel; // Linear acceleration in imu frame [m/s^2]
  struct FloatEulers YprU; // Attitude uncertainty, 1sigma, float, [degrees], yaw, pitch, roll
  uint16_t ins_status; // see page 122 of VN-200 datasheet
  uint8_t mode; // 0-not tracking, 1 - poor performance, 2- OK
  uint8_t err; // see page 122 of VN-200 datasheet
  struct FloatVect3 vel_body; //The estimated velocity in the imu frame, given in m/s.
};


// global INS state
extern struct InsVectornav ins_vn;

extern void ins_vectornav_init(void);
extern void ins_vectornav_register(void);

void ins_vectornav_read_message(void);
void ins_vectornav_check_status(void);
void ins_vectornav_set_sacc(void);
void ins_vectornav_set_pacc(void);
void ins_vectornav_propagate(void);
void ins_vectornav_yawPitchRoll_to_attitude(struct FloatEulers *vn_attitude);
void ins_init_origin_from_flightplan(void);

#endif /* INS_VECTORNAV_H */
