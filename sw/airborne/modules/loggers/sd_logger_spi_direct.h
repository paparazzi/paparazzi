/*
 * Copyright (C) 2015 Bart Slinger <bartslinger@gmail.com>
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
 *
 */

/** @file modules/loggers/sd_logger.h
 *  @brief Module for real time logging using an SD Card in SPI mode.
 */

#ifndef SD_LOGGER_H_
#define SD_LOGGER_H_

#include "peripherals/sdcard_spi.h"

#define SD_LOGGER_BUFFER_OFFSET 1           /**< Byte reserved for the data token of a multiwrite packet, only applicable to output buffer */
#define SD_LOGGER_PACKET_SIZE 52            /**< Number of bytes in the LogPacket struct */
#define SD_LOGGER_PACKETS_PER_BLOCK (SD_BLOCK_SIZE/SD_LOGGER_PACKET_SIZE)
/**< Number of packets per block */
#define SD_LOGGER_BLOCK_PREAMBLE_SIZE 4     /**< 32-bit number that contains the unique_id in each block */

enum SdLoggerStatus {
  SdLogger_UnInit,                          /**< SD logger is not initialized */
  SdLogger_Initializing,                    /**< Initializing the SD Card */
  SdLogger_Idle,                            /**< Idle state, ready to accept commands */
  SdLogger_Error,                           /**< Something failed */
  SdLogger_BeforeLogging,                   /**< Clear status block before start with logging */
  SdLogger_Logging,                         /**< Recording data */
  SdLogger_StopLogging,                     /**< Stop multiwrite on the sd card with stop token */
  SdLogger_WriteStatusPacket,               /**< After stopping, writing summary info to block 0 */
  SdLogger_DataAvailable,                   /**< In this state, the sdcard input buffer has data from address block_addr */
  SdLogger_ReadingBlock,                    /**< Temporary status when reading block until the callback */
};

enum SdLoggerCommand {
  SdLoggerCmd_Nothing,                      /**< Do nothing */
  SdLoggerCmd_StartLogging,                 /**< Start logging data */
  SdLoggerCmd_StopLogging,                  /**< Stop logging data */
  SdLoggerCmd_RequestStatusPacket,          /**< Requesting the status packet (first packet on block 0) */
};

/* Size of this struct (in bytes) should be equal to SD_LOGGER_PACKET_SIZE */
struct LogPacket {
  uint32_t time;
  int32_t data_1;
  int32_t data_2;
  int32_t data_3;
  int32_t data_4;
  int32_t data_5;
  int32_t data_6;
  int32_t data_7;
  int32_t data_8;
  int32_t data_9;
  int32_t data_10;
  int32_t data_11;
  int32_t data_12;
};

struct SdLogger {
  enum SdLoggerStatus status;               /**< Status of the sd logger */
  enum SdLoggerCommand cmd;                 /**< Command set by remote application */
  struct LogPacket packet;                  /**< Packet to sent to groundstation */
  uint32_t unique_id;                       /**< Unique number to identify written (and missing!) blocks */
  uint32_t block_addr;                      /**< Block address of the SD card where data is being logged to or being read from*/
  uint16_t buffer_addr;                     /**< Logging location in sd output buffer */
  uint32_t packet_count;                    /**< Number of packets saved */
  uint32_t error_count;                     /**< Count number of errors during logging phase */
  uint32_t request_id;                      /**< Setable from external application to request log packet */
  uint8_t timeout_counter;                  /**< Number of attempts to read a data block */
};

extern struct SdLogger sdlogger;

/* Public functions */
extern void sd_logger_start(void);
extern void sd_logger_periodic(void);
extern void sd_logger_command(void);
extern void sd_logger_stop(void);

/* Private functions */
extern void sd_logger_statusblock_ready(void);
extern void sd_logger_packetblock_ready(void);
extern void sd_logger_send_packet_from_buffer(uint16_t buffer_idx);
extern void sd_logger_int32_to_buffer(const int32_t value, uint8_t *target);
extern void sd_logger_uint32_to_buffer(const uint32_t value, uint8_t *target);
extern int32_t sd_logger_get_int32(uint8_t *target);
extern uint32_t sd_logger_get_uint32(uint8_t *target);

#endif /* SD_LOGGER_H_ */
