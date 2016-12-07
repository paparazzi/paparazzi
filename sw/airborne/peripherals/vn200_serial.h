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
 * @file vn200_serial.h
 *
 * Vectornav VN-200 INS subsystem
 *
 * @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 */
#ifndef VN200_SERIAl_H
#define VN200_SERIAl_H

#include "std.h"
#include "mcu_periph/uart.h"

// Geodetic / Math
#include "math/pprz_algebra.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_isa.h"

/*
 * Defines for the serial communication
 */
#define VN_SYNC 0xFA
#define VN_OUTPUT_GROUP 0x39
#define VN_GROUP_BYTES 8

#define VN_BUFFER_SIZE 512
#define VN_HEADER_SIZE 9
#define VN_PAYLOAD_SIZE 144


enum VNMsgStatus {
  VNMsgSync,
  VNMsgHeader,
  VNMsgGroup,
  VNMsgData,
  VNMsgCheck
};

struct VNPacket {
  bool  msg_available;
  uint32_t chksm_error;
  uint32_t hdr_error;
  uint8_t msg_buf[VN_BUFFER_SIZE];
  enum VNMsgStatus status;
  uint8_t  msg_idx;
  uint16_t datalength;
  uint16_t overrun_error;
  uint16_t noise_error;
  uint16_t framing_error;
  uint16_t calc_chk;
  uint16_t rec_chk;
  uint16_t counter;
};

enum VNStatus {
  VNNotTracking,
  VNOutOfSpecs,
  VNOK
};

struct VNData {
  uint64_t nanostamp; // Timestamp [nanoseconds] since startup
  float timestamp; ///< Time since VN startup [s]
  float ypr[3]; ///< yaw, pitch, roll [deg]
  struct FloatEulers attitude; ///< Attitude, float, [rad], yaw, pitch, roll
  struct FloatVect3 accel; ///< Acceleration in the imu frame, m/s
  struct FloatRates gyro; ///< Rates in the imu frame m/s
  float pos_u[3]; ///< The current GPS position uncertainty in the North East Down (NED) coordinate frame, given in meters.
  float vel_u; ///< NED velocity uncertainty [m/s]
  struct FloatVect3 lin_accel; ///< Linear acceleration in imu frame [m/s^2]
  struct FloatEulers ypr_u; ///< Attitude uncertainty, 1sigma, float, [degrees], yaw, pitch, roll
  uint16_t ins_status; ///< see page 122 of VN-200 datasheet
  uint8_t mode; ///< 0-not tracking, 1 - poor performance, 2- OK
  uint8_t err; ///< see page 122 of VN-200 datasheet
  struct FloatVect3 vel_body; ///< The estimated velocity in the imu frame, given in m/s.
  uint64_t tow; ///< tow (in nanoseconds), uint64
  uint8_t num_sv; ///< number of visible satellites
  uint8_t gps_fix; ///< None|2D|3D
  double pos_lla[3]; // Lla [deg, deg, m above elipsoid]
  struct NedCoor_f vel_ned; ///< The estimated velocity in the North East Down (NED) frame, given in m/s.
};

void vn200_event(struct VNPacket *vnp);
void vn200_read_message(struct VNPacket *vnp, struct VNData *vndata);
void vn200_parse(struct VNPacket *vnp, uint8_t c);


#endif /* VN200_SERIAl_H */
