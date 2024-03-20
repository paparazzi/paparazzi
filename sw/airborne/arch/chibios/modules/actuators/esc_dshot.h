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
#include "modules/actuators/esc_dshot_config.h"
#if DSHOT_BIDIR
#include "modules/actuators/dshot_rpmCapture.h"
#include "modules/actuators/dshot_erps.h"
#endif


#ifndef DSHOT_CHANNEL_FIRST_INDEX
#define DSHOT_CHANNEL_FIRST_INDEX 0U
#endif

/** DMA buffer size and number of channels
 */
#define DSHOT_BIT_WIDTHS              16U
#define DSHOT_PRE_FRAME_SILENT_SYNC_BITS  2U
#define DSHOT_POST_FRAME_SILENT_SYNC_BITS 4U
#define DSHOT_DMA_BUFFER_SIZE	      (DSHOT_BIT_WIDTHS + \
				       DSHOT_PRE_FRAME_SILENT_SYNC_BITS + \
				       DSHOT_POST_FRAME_SILENT_SYNC_BITS )


#if STM32_DMA_SUPPORTS_DMAMUX
#define DSHOT_EMIT_STREAM_NX(tim)  STM32_DMAMUX1_TIM ## tim ## _UP
#define DSHOT_EMIT_STREAM(tim)  DSHOT_EMIT_STREAM_NX(tim)
#endif

/**
 * @brief  special values returned by dshotGetRpm function
 * @note   must be checked after each call to dshotGetRpm
 */
#define DSHOT_BIDIR_ERR_CRC		 UINT32_MAX
#define DSHOT_BIDIR_TLM_EDT		 (UINT32_MAX-1U)


/**
 * @brief   special value for index : send order to all channels
 * @note    could be used as index in dshotSetThrottle and
 *          dshotSendSpecialCommand functions
 */
#define DSHOT_ALL_MOTORS 255U

// /**
//  * @brief   Driver state machine possible states.
//  */
// typedef enum {
//   DSHOT_UNINIT = 0,                       /**< Not initialized.          */
//   DSHOT_STOP,                             /**< Stopped.                  */
//   DSHOT_READY,                            /**< Ready.                    */
//   DSHOT_ONGOING_TELEMETRY_QUERY,          /**< Transfering.              */
//   DSHOT_ERROR                             /**< Transfert error.          */
// } dshotstate_t;

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
  DSHOT_CMD_MOTOR_STOP = 0U,
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
  DSHOT_CMD_BIDIR_EDT_MODE_ON,
  DSHOT_CMD_BIDIR_EDT_MODE_OFF,
  DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20U,
  DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21U,
  DSHOT_CMD_LED0_ON, // BLHeli32 only
  DSHOT_CMD_LED1_ON, // BLHeli32 only
  DSHOT_CMD_LED2_ON, // BLHeli32 only
  DSHOT_CMD_LED3_ON, // BLHeli32 only
  DSHOT_CMD_LED0_OFF, // BLHeli32 only
  DSHOT_CMD_LED1_OFF, // BLHeli32 only
  DSHOT_CMD_LED2_OFF, // BLHeli32 only
  DSHOT_CMD_LED3_OFF, // BLHeli32 only
  DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF = 30U, // KISS audio Stream mode on/Off
  DSHOT_CMD_SILENT_MODE_ON_OFF = 31U, // KISS silent Mode on/Off
  DSHOT_CMD_MAX = 47U,
  DSHOT_MIN_THROTTLE
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
    }
      __attribute__ ((__packed__));
    uint8_t rawData[9];
  };
  uint8_t  crc8;
}  __attribute__ ((__packed__)) DshotTelemetryFrame ;


/**
 * @brief   telemetry with timestamp
 */
typedef struct {
  DshotTelemetryFrame frame; // fields shared by serial telemetry and EDT
  uint8_t  stress; // EDT additionnal field
  uint8_t  status; // EDT additionnal field
  systime_t	      ts; // timestamp of last succesfull received frame
}  DshotTelemetry ;


typedef union {
#if DSHOT_AT_LEAST_ONE_32B_TIMER
  uint32_t widths32[DSHOT_DMA_BUFFER_SIZE][DSHOT_CHANNELS];
#endif
  uint16_t widths16[DSHOT_DMA_BUFFER_SIZE][DSHOT_CHANNELS];
} DshotDmaBuffer;   // alignment to satisfy dma requirement

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
#if STM32_DMA_SUPPORTS_DMAMUX
  uint8_t	dmamux;
#else
  uint8_t	dma_channel;
#endif

  /**
   * @brief PWM driver that feed up to 4 dshot lines
   */
  PWMDriver *pwmp;

  /**
   * @brief if non null : dshot telemetry serial driver
   */
  SerialDriver	*tlm_sd;

  /**
   * @brief dshot dma buffer, should be defined in a non Dcached region
   */
  DshotDmaBuffer *dma_buf;

#if DSHOT_BIDIR
 /**
   * @brief : DshotRpmCapture configuration structure when DSHOT_BIDIR is enabled
   */
  DshotRpmCaptureConfig dma_capt_cfg;
#endif

#if DSHOT_SPEED == 0
 /**
   * @brief	dynamic dshot speed, when speed id not known at compilation
   * @note	incompatible with BIDIR
   */
  uint16_t speed_khz;
#endif

#if __DCACHE_PRESENT
  /**
   * @brief   DMA memory is in a cached section and need to be flushed
   */
  bool		 dcache_memory_in_use;
#endif
} DSHOTConfig;






void     dshotStart(DSHOTDriver *driver, const DSHOTConfig *config);
void     dshotStop(DSHOTDriver *driver);
void     dshotSetThrottle(DSHOTDriver *driver, const uint8_t index, const uint16_t throttle);
void     dshotSendFrame(DSHOTDriver *driver);
void     dshotSendThrottles(DSHOTDriver *driver, const uint16_t throttles[DSHOT_CHANNELS]);
void     dshotSendSpecialCommand(DSHOTDriver *driver, const uint8_t index, const dshot_special_commands_t specmd);

uint32_t dshotGetCrcErrorCount(const DSHOTDriver *driver);
uint32_t dshotGetTelemetryFrameCount(const DSHOTDriver *driver);
DshotTelemetry dshotGetTelemetry(DSHOTDriver *driver, const uint32_t index);
#if DSHOT_BIDIR
uint32_t dshotGetEperiod(DSHOTDriver *driver, const uint32_t index);
uint32_t dshotGetRpm(DSHOTDriver *driver, const uint32_t index);
#endif


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
    uint16_t crc:4;
    uint16_t telemetryRequest:1;
    uint16_t throttle:11;
  };
  uint16_t rawFrame;
} DshotPacket;


typedef struct {
  DshotPacket       dp[DSHOT_CHANNELS];
  DshotTelemetry    dt[DSHOT_CHANNELS];
  mutex_t	    tlmMtx[DSHOT_CHANNELS];
  volatile bool	    onGoingQry;
  uint8_t  currentTlmQry;
} DshotPackets;


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
   * @brief number of sucessful telemetry frame received
   */
  uint32_t tlm_frame_nb;
  
#if DSHOT_SPEED == 0
  uint16_t bit0Duty;
  uint16_t bit1Duty;
#endif
#if DSHOT_BIDIR
  /**
   * @brief object managing capture of erpm frame 
   *	    using timer in input capture mode dans one dma stream 
   *        for each channels
   */
  DshotRpmCapture rpm_capture;
  /**
   * @brief object managing decoding of erpm frame
   */
  DshotErps	  erps;
  /**
   * @brief half decoded rpms frame
   */
  uint32_t	  rpms_frame[DSHOT_CHANNELS];
#endif
  /**
   * @brief stack working area for dshot telemetry thread
   */
  THD_WORKING_AREA(waDshotTlmRec, 512);

  /**
   * @brief object managing dma control frame for outgoing command
   */
  DshotPackets dshotMotors;
};

