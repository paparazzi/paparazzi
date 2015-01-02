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
 */

/**
 * @file peripherals/sst25vfxxxx.h
 * Driver for the SST25Vxxxx flash chips
 */

#ifndef SST25VFXXXX_H
#define SST25VFXXXX_H

#include "mcu_periph/spi.h"

/* Register defines */
#define SST25VFXXXX_READ                0x03
#define SST25VFXXXX_HGIH_SPEAD_READ     0x0B
#define SST25VFXXXX_ERASE_4K            0x20
#define SST25VFXXXX_ERASE_32K           0x52
#define SST25VFXXXX_ERASE_64K           0xD8
#define SST25VFXXXX_ERASE_CHIP          0x60
#define SST25VFXXXX_BYTE_PROG           0x02
#define SST25VFXXXX_AAI_PROG            0xAD
#define SST25VFXXXX_RDSR                0x05
#define SST25VFXXXX_EWSR                0x50
#define SST25VFXXXX_WRSR                0x01
#define SST25VFXXXX_WREN                0x06
#define SST25VFXXXX_WRDI                0x04
#define SST25VFXXXX_RDID                0x90
#define SST25VFXXXX_JEDEC_ID            0x9F
#define SST25VFXXXX_EBSY                0x70
#define SST25VFXXXX_DBSY                0x80

/* The different statuses the SST25VFxxxx chip can be in */
enum SST25VFxxxxStatus {
  SST25VFXXXX_UNINIT,                  /**< The chip isn't initialized */
  SST25VFXXXX_IDLE,                    /**< The chip is idle and can be used */
  SST25VFXXXX_READ_ID,                 /**< The chip is busy with getting the chip ID */
  SST25VFXXXX_WRITE_EN,                /**< The chip is busy enabeling writing to blocks */
  SST25VFXXXX_CHIP_ERASE,              /**< The chip is busy erasing itself */
  SST25VFXXXX_WRITE_BYTES,             /**< The chip is busy writing bytes */
  SST25VFXXXX_READ_BYTES,              /**< The chip is busy reading bytes */
};

/* The structure for the SST25VFxxxx chip that handles all the buffers and requests */
struct SST25VFxxxx {
  volatile enum SST25VFxxxxStatus status;   /**< The status of the SST25VFxxxx flash chip */
  uint8_t status_idx;                       /**< The counter of substatuses */
  struct spi_periph *spi_p;                 /**< The SPI peripheral for the connection */
  struct spi_transaction spi_t;             /**< The SPI transaction used for the writing and reading of registers */
  uint8_t input_buf[16];                    /**< The input buffer for the SPI transaction */
  uint8_t output_buf[16];                   /**< The output buffer for the SPI transaction */
  uint32_t flash_addr;                      /**< The flash address to write at */

  uint8_t *transfer_buf;                    /**< The transfer buffer */
  uint8_t transfer_idx;                     /**< The transfer idx is used for counting input/output bytes */
  uint8_t transfer_length;                  /**< The transfer buffer length */
};

void sst25vfxxxx_init(struct SST25VFxxxx *sst, struct spi_periph *spi_p, const uint8_t slave_idx, SPICallback spi_cb);
void sst25vfxxxx_after_cb(struct SST25VFxxxx *sst);
void sst25vfxxxx_read_id(struct SST25VFxxxx *sst);
void sst25vfxxxx_block_write_en(struct SST25VFxxxx *sst);
void sst25vfxxxx_chip_erase(struct SST25VFxxxx *sst);
void sst25vfxxxx_write(struct SST25VFxxxx *sst, uint8_t *transfer_buffer, uint8_t transfer_length);
void sst25vfxxxx_read(struct SST25VFxxxx *sst, uint8_t *transfer_buffer, uint8_t transfer_length);

#endif /* SST25VFXXXX_H */
