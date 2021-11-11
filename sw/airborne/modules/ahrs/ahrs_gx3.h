/*
 * Copyright (C) 2013 Michal Podhradsky
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
 * @file ahrs_gx3.h
 *
 * Driver for Microstrain GX3 IMU/AHRS subsystem
 *
 * Takes care of configuration of the IMU, communication and parsing
 * the received packets. See GX3 datasheet for configuration options.
 *
 * @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 */
#ifndef AHRS_GX3_H
#define AHRS_GX3_H

#include "generated/airframe.h"
#include "modules/imu/imu.h"
#include "modules/ins/ins.h"
#include "modules/gps/gps.h"
#include "mcu_periph/uart.h"

#include "state.h"
#include "led.h"

#define GX3_MAX_PAYLOAD 128
#define GX3_MSG_LEN 67
#define GX3_HEADER 0xC8
#define GX3_MIN_FREQ 300

#define IMU_GX3_LONG_DELAY 4000000

extern void gx3_packet_read_message(void);
extern void gx3_packet_parse(uint8_t c);

struct GX3Packet {
  bool  msg_available;
  uint32_t chksm_error;
  uint32_t hdr_error;
  uint8_t msg_buf[GX3_MAX_PAYLOAD];
  uint8_t status;
  uint8_t msg_idx;
};

enum GX3PacketStatus {
  GX3PacketWaiting,
  GX3PacketReading
};


//AHRS
struct AhrsGX3 {
  struct FloatQuat   ltp_to_imu_quat;  ///< Rotation from LocalTangentPlane to IMU frame as quaternions
  float mag_offset;                    ///< Difference between true and magnetic north

  struct GX3Packet packet;       ///< Packet struct
  float freq;                     ///< data frequency
  uint16_t chksm;                 ///< aux variable for checksum
  uint32_t time;                  ///< GX3 time stamp
  uint32_t ltime;                 ///< aux time stamp
  struct FloatVect3 accel;        ///< measured acceleration in IMU frame
  struct FloatRates rate;         ///< measured angular rates in IMU frame
  struct FloatRMat  rmat;         ///< measured attitude in IMU frame (rotational matrix)
  bool is_aligned;
};

extern struct AhrsGX3 ahrs_gx3;

#ifndef PRIMARY_AHRS
#define PRIMARY_AHRS ahrs_gx3
#endif

extern void ahrs_gx3_init(void);
extern void ahrs_gx3_align(void);
extern void ahrs_gx3_register(void);
extern void ahrs_gx3_publish_imu(void);

extern void imu_gx3_init(void);
extern void imu_gx3_periodic(void);
extern void imu_gx3_event(void);

#endif /* AHRS_GX3_H*/
