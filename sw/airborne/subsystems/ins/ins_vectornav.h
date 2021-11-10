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
#include "modules/gps/gps.h"
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

// Systime
#include "mcu_periph/sys_time.h"

// Abi
#include "modules/core/abi.h"


#if !defined INS_VN_BODY_TO_IMU_PHI && !defined INS_VN_BODY_TO_IMU_THETA && !defined INS_VN_BODY_TO_IMU_PSI
#define INS_VN_BODY_TO_IMU_PHI   0
#define INS_VN_BODY_TO_IMU_THETA 0
#define INS_VN_BODY_TO_IMU_PSI   0
#endif


// Ins implementation state (fixed point)
struct InsVectornav {
  struct LtpDef_i  ltp_def; // initial position
  bool           ltp_initialized; // status indicator

  // output LTP NED for telemetry messages
  struct NedCoor_i ltp_pos_i;
  struct NedCoor_i ltp_speed_i;
  struct NedCoor_i ltp_accel_i;
  struct LlaCoor_f lla_pos;
  struct NedCoor_f ltp_accel_f;

  // baro [height above ground]
  float baro_z;  ///< z-position calculated from baro in meters (z-down)
  float qfe;

  // Packet data
  struct VNPacket vn_packet;///< Packet struct
  struct VNData vn_data; ///< Data struct
  enum VNStatus vn_status;  ///< VN status
  uint16_t vn_rate;            ///< data frequency

  // in fixed point for sending as ABI and telemetry msgs
  struct Int32Vect3 accel_i;
  struct Int32Rates gyro_i;

  /** body_to_imu rotation */
  struct OrientationReps body_to_imu;
};


// global INS state
extern struct InsVectornav ins_vn;

extern void ins_vectornav_init(void);
extern void ins_vectornav_event(void);
extern void ins_vectornav_set_sacc(void);
extern void ins_vectornav_set_pacc(void);
extern void ins_vectornav_propagate(void);
extern void ins_vectornav_monitor(void);

#endif /* INS_VECTORNAV_H */
