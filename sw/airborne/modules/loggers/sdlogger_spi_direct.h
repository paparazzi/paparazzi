/*
 * Copyright (C) Bart Slinger
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
 * @file "modules/loggers/sdlogger_spi_direct.h"
 * @author Bart Slinger
 * SPI SD Logger that saves pprzlog messages to SD Card.
 */

#ifndef SDLOGGER_SPI_H
#define SDLOGGER_SPI_H

#define SDLOGGER_BUFFER_SIZE 128

#include "mcu_periph/link_device.h"
#include "peripherals/sdcard_spi.h"

enum SDLoggerStatus {
  SDLogger_UnInit,
  SDLogger_Error,
  SDLogger_Initializing,
  SDLogger_RetreivingIndex,
  SDLogger_Ready,
  SDLogger_Logging,
  SDLogger_LoggingFinalBlock,
  SDLogger_StoppedLogging,
  SDLogger_GettingIndexForUpdate,
  SDLogger_UpdatingIndex,
  SDLogger_GettingIndexForDownload,
  SDLogger_Downloading
};

struct sdlogger_spi_periph{
  enum SDLoggerStatus status;
  uint32_t next_available_address;
  uint8_t last_completed;
  uint16_t sdcard_buf_idx;
  uint8_t buffer[SDLOGGER_BUFFER_SIZE];
  uint8_t idx;
  uint32_t log_len;
  uint8_t command;
  uint8_t download_id;
  uint32_t download_address;
  uint32_t download_length;
  struct link_device device;
};

extern struct sdlogger_spi_periph sdlogger_spi;

extern void sdlogger_spi_direct_init(void);
extern void sdlogger_spi_direct_periodic(void);
extern void sdlogger_spi_direct_start(void);
extern void sdlogger_spi_direct_stop(void);

extern void sdlogger_spi_direct_index_received(void);
extern void sdlogger_spi_direct_multiwrite_written(void);
extern void sdlogger_spi_direct_command(void);

extern bool_t sdlogger_spi_direct_check_free_space(struct sdlogger_spi_periph *p, uint8_t len);
extern void sdlogger_spi_direct_put_byte(struct sdlogger_spi_periph *p, uint8_t data);
extern void sdlogger_spi_direct_send_message(void *p);
extern int sdlogger_spi_direct_char_available(void *p);
extern uint8_t sdlogger_spi_direct_get_byte(void *p);

#endif

