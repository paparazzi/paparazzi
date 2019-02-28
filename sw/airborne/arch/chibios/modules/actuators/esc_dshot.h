/*
 * Copyright (C) 2018 Alexandre Bustico <alexandre.bustico@enac.fr>
 *                    Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file    esc_dshot.h
 * @brief   DSHOT driver based on ChibiOS
 *
 * @author Alexandre Bustico
 * @maintainer Gautier Hattenberger <gautier.hattenberger@enac.fr>
 */

#pragma once

#include <ch.h>
#include <hal.h>
#include "mcu_periph/hal_stm32_dma.h"

/**
 *  By default enable the possibility to mix 16 and 32 bits timer
 */
#ifndef DSHOT_AT_LEAST_ONE_32B_TIMER
#define DSHOT_AT_LEAST_ONE_32B_TIMER TRUE
#endif

/** DMA buffer size and number of channels
 */
#define DSHOT_BIT_WIDTHS              16
#define DSHOT_FRAME_SILENT_SYNC_BITS  4
#define DSHOT_DMA_BUFFER_SIZE         (DSHOT_BIT_WIDTHS+DSHOT_FRAME_SILENT_SYNC_BITS)
#define DSHOT_CHANNELS                4 // depend on the number of channels per timer

/**
 * @brief   special value for index : send order to all channels
 * @note    could be used as index in dshotSetThrottle and
 *          dshotSendSpecialCommand functions
 */
#define DSHOT_ALL_MOTORS 255

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  DSHOT_UNINIT = 0,                       /**< Not initialized.          */
  DSHOT_STOP,                             /**< Stopped.                  */
  DSHOT_READY,                            /**< Ready.                    */
  DSHOT_ONGOING_TELEMETRY_QUERY,          /**< Transfering.              */
  DSHOT_ERROR                             /**< Transfert error.          */
} dshotstate_t;

/*
  DshotSettingRequest (KISS24). Spin direction,
  3d and save Settings require 10 requests.. and the
  TLM Byte must always be high if 1-47 are used to send settings

  3D Mode:
  0 = stop
  48   (low) - 1047 (high) -> negative direction
  1048 (low) - 2047 (high) -> positive direction
 */

/**
 * @brief   DSHOT special commands (0-47) for KISS and BLHELI ESC
 * @note    commands 48-2047 are used to send motor power
 */
typedef enum {
  DSHOT_CMD_MOTOR_STOP = 0,
  DSHOT_CMD_BEACON1,
  DSHOT_CMD_BEACON2,
  DSHOT_CMD_BEACON3,
  DSHOT_CMD_BEACON4,
  DSHOT_CMD_BEACON5,
  DSHOT_CMD_ESC_INFO, // V2 includes settings
  DSHOT_CMD_SPIN_DIRECTION_1,
  DSHOT_CMD_SPIN_DIRECTION_2,
  DSHOT_CMD_3D_MODE_OFF,
  DSHOT_CMD_3D_MODE_ON,
  DSHOT_CMD_SETTINGS_REQUEST, // Currently not implemented
  DSHOT_CMD_SAVE_SETTINGS,
  DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
  DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,
  DSHOT_CMD_LED0_ON, // BLHeli32 only
  DSHOT_CMD_LED1_ON, // BLHeli32 only
  DSHOT_CMD_LED2_ON, // BLHeli32 only
  DSHOT_CMD_LED3_ON, // BLHeli32 only
  DSHOT_CMD_LED0_OFF, // BLHeli32 only
  DSHOT_CMD_LED1_OFF, // BLHeli32 only
  DSHOT_CMD_LED2_OFF, // BLHeli32 only
  DSHOT_CMD_LED3_OFF, // BLHeli32 only
  DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF = 30, // KISS audio Stream mode on/Off
  DSHOT_CMD_SILENT_MODE_ON_OFF = 31, // KISS silent Mode on/Off
  DSHOT_CMD_MAX = 47
} dshot_special_commands_t;

/**
 * @brief   telemetry packed as sent by some KISS ESC
 * @note    if other ESC use different binary representation in the future
 *          we'll have to add a little bit abstraction here
 */
typedef struct {
  union {
    struct {
      uint8_t  temp;
      uint16_t voltage;
      uint16_t current;
      uint16_t consumption;
      uint16_t rpm;
    } __attribute__((__packed__, scalar_storage_order("big-endian")));
    uint8_t rawData[9];
  };
  uint8_t  crc8;
}  __attribute__((__packed__)) DshotTelemetry ;


/**
 * @brief   Type of a structure representing an DSHOT driver.
 */
typedef struct DSHOTDriver DSHOTDriver;


/**
 * @brief   DSHOT  Driver configuration structure.
 */
typedef struct  {
  /**
   * @brief : dma stream associated with pwm timer used to generate dshot output
   */
  uint32_t  dma_stream;

  /**
   * @brief : dma channel associated with pwm timer used to generate dshot output
   */
  uint8_t dma_channel;

  /**
   * @brief PWM driver that feed up to 4 dshot lines
   */
  PWMDriver *pwmp;

  /**
   * @brief if non null : dshot telemetry serial driver
   */
  SerialDriver  *tlm_sd;
} DSHOTConfig;

void     dshotStart(DSHOTDriver *driver, const DSHOTConfig *config);
void     dshotSetThrottle(DSHOTDriver *driver, const uint8_t index, const uint16_t throttle);
void     dshotSendFrame(DSHOTDriver *driver);
void     dshotSendThrottles(DSHOTDriver *driver, const uint16_t throttles[DSHOT_CHANNELS]);
void     dshotSendSpecialCommand(DSHOTDriver *driver, const uint8_t index, const dshot_special_commands_t specmd);

uint32_t dshotGetCrcErrorsCount(DSHOTDriver *driver);
const DshotTelemetry *dshotGetTelemetry(const DSHOTDriver *driver, const uint32_t index);


/*
#                 _ __           _                    _
#                | '_ \         (_)                  | |
#                | |_) |  _ __   _   __   __   __ _  | |_     ___
#                | .__/  | '__| | |  \ \ / /  / _` | | __|   / _ \
#                | |     | |    | |   \ V /  | (_| | \ |_   |  __/
#                |_|     |_|    |_|    \_/    \__,_|  \__|   \___|
*/

typedef union {
  struct {
    uint16_t crc: 4;
    uint16_t telemetryRequest: 1;
    uint16_t throttle: 11;
  };
  uint16_t rawFrame;
} DshotPacket;

typedef struct {
  DshotPacket       dp[DSHOT_CHANNELS];
  DshotTelemetry    dt[DSHOT_CHANNELS];
  volatile uint8_t  currentTlmQry;
  volatile bool     onGoingQry;
} DshotPackets;

typedef union {
  uint16_t widths16[DSHOT_DMA_BUFFER_SIZE][DSHOT_CHANNELS];
#if DSHOT_AT_LEAST_ONE_32B_TIMER
  uint32_t widths32[DSHOT_DMA_BUFFER_SIZE][DSHOT_CHANNELS];
#endif
} DshotDmaBuffer __attribute__((aligned(16)));   // alignment to satisfy dma requirement

/**
 * @brief   DSHOT  driver structure.
 */
struct  DSHOTDriver {
  /**
   * @brief DMA config associated with pwm timer
   */
  const DSHOTConfig *config;

  /**
   * @brief DMA config associated with pwm timer
   */
  DMAConfig dma_conf;

  /**
   * @brief PWM config associated with pwm timer
   */
  PWMConfig pwm_conf;

  /**
   * @brief DMA driver associated with pwm timer
   */
  DMADriver dmap;

  /**
   * @brief mailbox buffer for dshot telemetry thread
   */
  msg_t  _mbBuf[1];

  /**
   * @brief mailbox for dshot telemetry thread
   */
  mailbox_t mb;

  /**
   * @brief number of crc errors
   */
  uint32_t crc_errors;

  /**
   * @brief stack working area for dshot telemetry thread
   */
  THD_WORKING_AREA(waDshotTlmRec, 512);

  DshotPackets dshotMotors;
  DshotDmaBuffer dsdb;
};

