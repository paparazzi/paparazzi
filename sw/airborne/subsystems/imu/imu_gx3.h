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
 * @file imu_gx3.h
 *
 * Driver for Microstrain GX3 IMU/AHRS subsystem
 *
 * Takes care of configuration of the IMU, communication and parsing
 * the received packets. See GX3 datasheet for configuration options.
 *
 * @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 */
#ifndef IMU_GX3_H
#define IMU_GX3_H

#include "subsystems/imu.h"
#include "subsystems/ahrs.h"
#include "subsystems/ins.h"

#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time.h"

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

#if USE_CHIBIOS_RTOS
#define GX3_QUEUE_SIZE 5
#define CH_THREAD_AREA_IMU_RX 1024
#define INIT_IMU_THREAD 1
extern __attribute__((noreturn)) msg_t thd_imu_rx(void *arg);
#endif /* USE_CHIBIOS_RTOS */

#define IMU_GX3_LONG_DELAY 8000000

extern void gx3_packet_read_message(void);
extern void gx3_packet_parse(uint8_t c);
extern void imu_align(void);

struct GX3Packet {
  bool_t  msg_available;
  uint32_t chksm_error;
  uint32_t hdr_error;
  uint8_t msg_buf[GX3_MAX_PAYLOAD];
  uint8_t  status;
  uint8_t  msg_idx;
};

#if USE_CHIBIOS_RTOS
struct GX3Queue {
  uint8_t front;
  uint8_t rear;
  uint8_t queue_buf[GX3_QUEUE_SIZE][GX3_MSG_LEN];
  uint8_t length;
};
#endif /* USE_CHIBIOS_RTOS */

enum GX3PacketStatus {
  GX3PacketWaiting,
  GX3PacketReading,
  GX3PacketFull,
};

enum GX3Status {
  GX3Uninit,
  GX3StopContinuousMode,
  GX3StopContinuousMode_OK,
  GX3SamplingSettings,
  GX3SamplingSettings_OK,
  GX3ContinuousMode,
  GX3Running
};

struct ImuGX3 {
  struct GX3Packet gx3_packet;        ///< Packet struct
  enum GX3Status gx3_status;          ///< GX3 status
  float gx3_freq;                     ///< data frequency
  uint16_t gx3_chksm;                 ///< aux variable for checksum
  uint32_t gx3_time;                  ///< GX3 time stamp
  uint32_t gx3_ltime;                 ///< aux time stamp
  struct FloatRMat  rmat;         ///< measured attitude in IMU frame (rotational matrix)

#if USE_CHIBIOS_RTOS
  uint32_t ch_ltime;                  ///< packet time stamp
  uint32_t ch_time;                   ///< aux time stamp
  float ch_freq;                     ///< packet frequency
  struct GX3Queue queue;              ///< packet queue
  uint16_t freq_err;                  ///< check timing errors
  uint8_t gx3_data_buffer[GX3_MSG_LEN];//< packet to be read
  uint8_t data_valid;                  //< new data ready
#endif /* USE_CHIBIOS_RTOS */
};

extern struct ImuGX3 imu_gx3;

#if USE_CHIBIOS_RTOS
static inline void ImuEvent(void (* _gyro_handler)(void), void (* _accel_handler)(void), void (* _mag_handler)(void)) {
  if (imu_gx3.data_valid) {
    _gyro_handler();
    _accel_handler();
    _mag_handler();
    imu_gx3.data_valid = FALSE;
  }
}
#else
static inline void ReadGX3Buffer(void) {
  while (uart_char_available(&GX3_PORT) && !imu_gx3.gx3_packet.msg_available)
    gx3_packet_parse(uart_getch(&GX3_PORT));
}

static inline void ImuEvent(void (* _gyro_handler)(void), void (* _accel_handler)(void), void (* _mag_handler)(void)) {
  if (uart_char_available(&GX3_PORT)) {
    ReadGX3Buffer();
  }
  if (imu_gx3.gx3_packet.msg_available) {
    gx3_packet_read_message();
    _gyro_handler();
    _accel_handler();
    _mag_handler();
    imu_gx3.gx3_packet.msg_available = FALSE;
  }
}
#endif /* USE_CHIBIOS_RTOS */

#endif /* IMU_GX3_H*/
