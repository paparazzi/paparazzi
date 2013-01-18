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
 * @file imu_um6.h
 *
 * Driver for CH Robotics UM6 IMU/AHRS subsystem
 *
 * Takes care of configuration of the IMU, communication and parsing
 * the received packets. See UM6 datasheet for configuration options.
 * Should be used with @ahrs_extern_euler AHRS subsystem.
 *
 * @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 */
#ifndef IMU_UM6_H
#define IMU_UM6_H

#include "generated/airframe.h"
#include "subsystems/imu.h"
#include "mcu_periph/uart.h"
#include "subsystems/ahrs.h"
#include "subsystems/ins.h"

#define __UM6Link(dev, _x) dev##_x
#define _UM6Link(dev, _x)  __UM6Link(dev, _x)
#define UM6Link(_x) _UM6Link(UM6_LINK, _x)

#define UM6Buffer() UM6Link(ChAvailable())

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

#define IMU_UM6_BUFFER_LENGTH 32
#define IMU_UM6_DATA_OFFSET 5
#define IMU_UM6_LONG_DELAY 4000000

#define IMU_UM6_COMMUNICATION_REG 0x00
#define IMU_UM6_MISC_CONFIG_REG 0x01
#define IMU_UM6_GET_FIRMWARE_CMD 0xAA
#define IMU_UM6_ZERO_GYROS_CMD 0xAC
#define IMU_UM6_RESET_EKF_CMD 0xAD  
#define IMU_UM6_GET_DATA 0xAE
#define IMU_UM6_SET_ACCEL_REF 0xAF
#define IMU_UM6_SET_MAG_REF 0xB0  

#define IMU_UM6_GYRO_PROC 0x5C
#define IMU_UM6_ACCEL_PROC 0x5E
#define IMU_UM6_MAG_PROC 0x60
#define IMU_UM6_EULER 0x62
#define IMU_UM6_QUAT 0x64

extern void UM6_packet_read_message(void);
extern void UM6_packet_parse(uint8_t c);

extern struct UM6Packet UM6_packet;

extern uint8_t PacketLength;
extern uint8_t PacketType;
extern uint8_t PacketAddr;

extern uint16_t chk_calc;
extern uint16_t chk_rec;

extern enum UM6Status UM6_status;
extern volatile uint8_t UM6_imu_available;

extern struct FloatEulers UM6_eulers;
extern struct FloatQuat UM6_quat;

struct UM6Packet {
  bool_t  msg_available;
  uint32_t chksm_error;
  uint32_t hdr_error;
  uint8_t msg_buf[IMU_UM6_BUFFER_LENGTH];
  uint8_t  status;
  uint8_t  msg_idx;
};

enum UM6PacketStatus {
    UM6PacketWaiting,
    UM6PacketReadingS,
    UM6PacketReadingN,
    UM6PacketReadingPT,
    UM6PacketReadingAddr,
    UM6PacketReadingData
};

enum UM6Status {
  UM6Uninit,
  UM6Running
};
		
#define imu_um6_event(_callback1, _callback2, _callback3) {				\
    if (UM6Buffer()) {							\
      ReadUM6Buffer();							\
    }									\
    if (UM6_packet.msg_available) {					\
      UM6_packet.msg_available = FALSE;					\
      UM6_packet_read_message();					\
      _callback1();			\
      _callback2();			\
      _callback3();     \
    } \
}

#define ReadUM6Buffer() {						\
    while (UM6Link(ChAvailable())&&!UM6_packet.msg_available)		\
      UM6_packet_parse(UM6Link(Getch()));				\
  }

#define ImuEvent(_gyro_handler, _accel_handler, _mag_handler) { \
  imu_um6_event(_gyro_handler, _accel_handler, _mag_handler); \
}

#endif /* IMU_UM6_H*/
