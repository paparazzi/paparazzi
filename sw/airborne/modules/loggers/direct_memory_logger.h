/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
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

/**
 * @file modules/loggers/direct_memory_logger.h
 * @brief Write logs directly to flash memory chips
 */

#ifndef DIRECT_MEMORY_LOGGER_H_
#define DIRECT_MEMORY_LOGGER_H_

#include "peripherals/sst25vfxxxx.h"

#define DML_BUF_SIZE        128  /**< The read buffer size */

/* The different statusses the Direct Memory Logger is able to be in */
enum DMLStatus {
  DML_INIT,                     /**< The DML is initializing */
  DML_IDLE,                     /**< The DML is idle */
  DML_ERASE,                    /**< The DML is busy erasing itself */
  DML_START,                    /**< The DML is starting the logger */
  DML_LOGGING,                  /**< The DML is busy logging */
  DML_STOP,                     /**< The DML is busy stopping */
  DML_READ,                     /**< The DML is busy starting read */
  DML_READING,                  /**< The DML is busy reading */
};

/* Contains all the direct memory information */
struct DirectMemoryLogger {
  struct SST25VFxxxx sst;                  /**< The memory chip */
  volatile enum DMLStatus status;          /**< The status of the Direct Memory Logger */
  uint8_t buffer[DML_BUF_SIZE];            /**< The buffer for writing and reading */
  uint32_t write_addr;
};

extern struct DirectMemoryLogger dml;
void direct_memory_logger_init(void);
void direct_memory_logger_periodic(void);
void direct_memory_logger_set(uint8_t val);

#endif /* DIRECT_MEMORY_LOGGER_H_ */
