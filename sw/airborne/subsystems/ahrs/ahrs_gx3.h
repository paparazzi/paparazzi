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
#include "subsystems/imu.h"
#include "subsystems/ahrs.h"
#include "subsystems/ins.h"
#include "subsystems/gps.h"
#include "mcu_periph/uart.h"
#include "subsystems/ahrs/ahrs_aligner.h"

#include "state.h"
#include "led.h"

#ifdef ImuScaleGyro
#undef ImuScaleGyro
#endif
#define ImuScaleGyro(_imu) {}

#ifdef ImuScaleAccel
#undef ImuScaleAccel
#endif
#define ImuScaleAccel(_imu) {}

#ifdef ImuScaleMag
#undef ImuScaleMag
#endif
#define ImuScaleMag(_imu) {}

#define GX3_MAX_PAYLOAD 128
#define GX3_MSG_LEN 67
#define GX3_HEADER 0xC8
#define GX3_MIN_FREQ 300

#define IMU_GX3_LONG_DELAY 4000000

extern void gx3_packet_read_message(void);
extern void gx3_packet_parse(uint8_t c);

struct GX3Packet {
  bool_t  msg_available;
  uint32_t chksm_error;
  uint32_t hdr_error;
  uint8_t msg_buf[GX3_MAX_PAYLOAD];
  uint8_t  status;
  uint8_t  msg_idx;
};

enum GX3PacketStatus {
  GX3PacketWaiting,
  GX3PacketReading
};

enum GX3Status {
  GX3Uninit,
  GX3Running
};

//AHRS
struct AhrsFloatQuat {
  struct FloatQuat   ltp_to_imu_quat;  ///< Rotation from LocalTangentPlane to IMU frame as quaternions
  float mag_offset;                    ///< Difference between true and magnetic north

  struct GX3Packet gx3_packet;       ///< Packet struct
  enum GX3Status gx3_status;          ///< GX3 status
  float gx3_freq;                     ///< data frequency
  uint16_t gx3_chksm;                 ///< aux variable for checksum
  uint32_t gx3_time;                  ///< GX3 time stamp
  uint32_t gx3_ltime;                 ///< aux time stamp
  struct FloatVect3 gx3_accel;        ///< measured acceleration in IMU frame
  struct FloatRates gx3_rate;         ///< measured angular rates in IMU frame
  struct FloatRMat  gx3_rmat;         ///< measured attitude in IMU frame (rotational matrix)
};

extern struct AhrsFloatQuat ahrs_impl;

static inline void ReadGX3Buffer(void) {
  while (uart_char_available(&GX3_PORT) && !ahrs_impl.gx3_packet.msg_available)
    gx3_packet_parse(uart_getch(&GX3_PORT));
}

static inline void ImuEvent(void (* _gyro_handler)(void), void (* _accel_handler)(void), void (* _mag_handler)(void)) {
  if (uart_char_available(&GX3_PORT)) {
    ReadGX3Buffer();
  }
  if (ahrs_impl.gx3_packet.msg_available) {
    gx3_packet_read_message();
    _gyro_handler();
    _accel_handler();
    _mag_handler();
    ahrs_impl.gx3_packet.msg_available = FALSE;
  }
}

#endif /* AHRS_GX3_H*/
