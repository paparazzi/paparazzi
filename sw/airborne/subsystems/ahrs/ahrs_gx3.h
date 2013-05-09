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
#include "mcu_periph/uart.h"
#include "subsystems/ahrs/ahrs_aligner.h"

#include "state.h"
#include "led.h"

#define __GX3Link(dev, _x) dev##_x
#define _GX3Link(dev, _x)  __GX3Link(dev, _x)
#define GX3Link(_x) _GX3Link(GX3_LINK, _x)

#define GX3Buffer() GX3Link(ChAvailable())

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

extern struct GX3_packet GX3_packet;
extern enum GX3Status GX3_status;

extern void GX3_packet_read_message(void);
extern void GX3_packet_parse(uint8_t c);

extern float GX3_freq;
extern uint16_t GX3_chksm;
extern uint16_t GX3_calcsm;
extern uint32_t GX3_time;

struct GX3_packet {
  bool_t  msg_available;
  uint32_t chksm_error;
  uint32_t hdr_error;
  uint8_t msg_buf[GX3_MAX_PAYLOAD];
  uint8_t  status;
  uint8_t  msg_idx;
};

enum GX36PacketStatus {
  GX3PacketWaiting,
  GX3PacketReading
};

enum GX3Status {
  GX3Uninit,
  GX3Running
};

//AHRS
struct AhrsFloatQuat {
  struct FloatEulers ltp_to_imu_euler; ///< Rotation from LocalTangentPlane to IMU frame as Euler angles
  struct FloatQuat   ltp_to_imu_quat;  ///< Rotation from LocalTangentPlane to IMU frame as quaternions
  struct FloatRates  imu_rate;         ///< Rotational velocity in IMU frame
  float mag_offset;
};

extern struct AhrsFloatQuat ahrs_impl;

static inline void ReadGX3Buffer(void) {
  while (GX3Link(ChAvailable()) && !GX3_packet.msg_available)
    GX3_packet_parse(GX3Link(Getch()));
}

static inline void ImuEvent(void (* _gyro_handler)(void), void (* _accel_handler)(void), void (* _mag_handler)(void)) {
  if (GX3Buffer()) {
    ReadGX3Buffer();
  }
  if (GX3_packet.msg_available) {
    GX3_packet_read_message();
    _gyro_handler();
    _accel_handler();
    _mag_handler();
    GX3_packet.msg_available = FALSE;
  }
}

#endif /* AHRS_GX3_H*/
