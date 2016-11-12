/*
 * Copyright (C) 2016 Michal Podhradsky, michal.podhradsky@aggiemail.usu.edu
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
 * @file ahrs_vectornav.h
 *
 * Vectornav VN-200 as AHRS
 *
 * @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 */
#ifndef AHRS_VECTORNAV_H
#define AHRS_VECTORNAV_H


// Peripheral
#include "peripherals/vn200_serial.h"

// Systime
#include "mcu_periph/sys_time.h"

// Math
#include "math/pprz_algebra.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_orientation_conversion.h"


// Ahrs implementation state
struct AhrsVectornav {
  // Packet data
  struct VNPacket vn_packet;///< Packet struct
  struct VNData vn_data; ///< Data struct
  enum VNStatus vn_status;  ///< VN status
  float vn_freq;            ///< data frequency
  uint16_t vn_chksm;         ///< aux variable for checksum

  // in fixed point for sending as ABI and telemetry msgs
  struct Int32Vect3 accel_i;
  struct Int32Rates gyro_i;

  /** body_to_imu rotation */
  struct OrientationReps body_to_imu;
};


// AHRS struct
extern struct AhrsVectornav ahrs_vn;

extern void ahrs_vectornav_init(void);
extern void ahrs_vectornav_event(void);
extern void ahrs_vectornav_propagate(void);

#endif /* INS_VECTORNAV_H */
