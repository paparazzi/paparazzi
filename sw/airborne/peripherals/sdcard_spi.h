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

/** @file peripherals/sdcard_spi.h
 *  @brief Interface for reading and writing blocks of data to and from an SD card directly over SPI.
 */

#ifndef SDCARD_H_
#define SDCARD_H_

#include "std.h"
#include "mcu_periph/spi.h"

/** SDCard Callback function.
 * If not NULL (or 0), call function
 */
typedef void (*SDCardCallback)(void);

#define SD_BLOCK_SIZE 512

enum SdResponseType {
  SdResponseNone,
  SdResponseR1,
  SdResponseR3,
  SdResponseR7
};

enum SDCardType {
  SDCardType_Unknown,
  SDCardType_MmcV3,
  SDCardType_SdV1,
  SDCardType_SdV2byte,
  SDCardType_SdV2block
};

enum SDCardStatus {
  SDCard_UnInit,                            /**< SDCard is not initialized */
  SDCard_Error,                             /**< An error has occured, sending debug message */
  SDCard_Idle,                              /**< Initialization sequence succesful */
  SDCard_Busy,                              /**< SDCard is busy with internal process */
  SDCard_BeforeDummyClock,                  /**< About to send dummy clock cycles to initialize spi mode */
  SDCard_SendingDummyClock,                 /**< Busy sending dummy clock cycles */
  SDCard_SendingCMD0,                       /**< Busy sending CMD0 */
  SDCard_ReadingCMD0Resp,                   /**< Reading R1 response to CMD0 byte by byte */
  SDCard_SendingCMD8,                       /**< Busy sending CMD8 */
  SDCard_ReadingCMD8Resp,                   /**< Reading R7 response to CMD8 byte by byte */
  SDCard_ReadingCMD8Parameter,              /**< Reading the 32-bit parameter after receiving 0x01 */
  SDCard_SendingACMD41v2,                   /**< Busy sending ACMD41 */
  SDCard_ReadingACMD41v2Resp,               /**< Reading R1 response to ACMD41 byte by byte */
  SDCard_SendingCMD58,                      /**< Busy sending CMD58 */
  SDCard_ReadingCMD58Resp,                  /**< Reading R3 response to CMD58 byte by byte */
  SDCard_ReadingCMD58Parameter,             /**< Reading the 32-bit parameter after receiving 0x00 from CMD58 */
  SDCard_SendingCMD16,                      /**< Busy sending CMD16 */
  SDCard_ReadingCMD16Resp,                  /**< Reading R1 response to CMD16 byte by byte */
  SDCard_SendingCMD24,                      /**< Busy sending CMD24 */
  SDCard_ReadingCMD24Resp,                  /**< Reading R1 response to CMD24 byte by byte */
  SDCard_BeforeSendingDataBlock,            /**< Start data block transfer */
  SDCard_SendingDataBlock,                  /**< Busy sending data block */
  SDCard_SendingCMD17,                      /**< Busy sending CMD17 (block read request) */
  SDCard_ReadingCMD17Resp,                  /**< Reading R1 response to CMD17 byte by byte */
  SDCard_WaitingForDataToken,               /**< Reading a byte each period until there is a data token or error */
  SDCard_ReadingDataBlock,                  /**< Busy reading data block */
  SDCard_SendingCMD25,                      /**< Busy sending CMD25 (multiwrite start command) */
  SDCard_ReadingCMD25Resp,                  /**< Reading R1 response to CMD25 byte by byte */
  SDCard_MultiWriteIdle,                    /**< CMD25 complete, ready to sent blocks */
  SDCard_MultiWriteWriting,                 /**< Busy with the SPI transfer in multiwrite */
  SDCard_MultiWriteBusy,                    /**< Busy flag after sending data block in multiwrite */
  SDCard_MultiWriteStopping,                /**< Busy sending the stop token */
};

struct SDCard {
  struct spi_periph *spi_p;                 /**< The SPI peripheral for the connection */
  struct spi_transaction spi_t;             /**< The SPI transaction used for the writing and reading of registers */
  volatile enum SDCardStatus status;        /**< The status of the SD card */
  uint8_t input_buf[SD_BLOCK_SIZE + 10];    /**< The input buffer for the SPI transaction */
  uint8_t output_buf[SD_BLOCK_SIZE + 10];   /**< The output buffer for the SPI transaction */
  uint8_t response_counter;                 /**< Response counter used at various locations */
  uint32_t timeout_counter;                 /**< Timeout counter used for initialization checks with ACMD41 */
  enum SDCardType card_type;                /**< Type of SDCard */
  SDCardCallback read_callback;             /**< Callback to call when read operation finishes */
};

extern struct SDCard sdcard1;

/* Public functions */
extern void sdcard_spi_init(struct SDCard *sdcard, struct spi_periph *spi_p, const uint8_t slave_idx);
extern void sdcard_spi_periodic(struct SDCard *sdcard);
extern void sdcard_spi_write_block(struct SDCard *sdcard, uint32_t addr);
extern void sdcard_spi_read_block(struct SDCard *sdcard, uint32_t addr, SDCardCallback callback);
extern void sdcard_spi_multiwrite_start(struct SDCard *sdcard, uint32_t addr);
extern void sdcard_spi_multiwrite_next(struct SDCard *sdcard);
extern void sdcard_spi_multiwrite_stop(struct SDCard *sdcard);

#endif // SDCARD_H_

