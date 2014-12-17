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
 * @file peripherals/sst25vfxxxx.c
 * Driver for the SST25Vxxxx flash chips
 */

#include "sst25vfxxxx.h"

/* Static function defines */

/**
 * Initializing the sst25vfxxxx chip
 */
void sst25vfxxxx_init(struct SST25VFxxxx *sst, struct spi_periph *spi_p, const uint8_t slave_idx, SPICallback spi_cb)
{
  /* Set spi_peripheral and start flash address */
  sst->spi_p = spi_p;
  sst->flash_addr = 0x0;

  /* Set the spi transaction */
  sst->spi_t.cpol = SPICpolIdleLow;
  sst->spi_t.cpha = SPICphaEdge1;
  sst->spi_t.dss = SPIDss8bit;
  sst->spi_t.bitorder = SPIMSBFirst;
  sst->spi_t.cdiv = SPIDiv32;

  sst->spi_t.input_length = 0;
  sst->spi_t.output_length = 0;
  sst->spi_t.input_buf = sst->input_buf;
  sst->spi_t.output_buf = sst->output_buf;
  sst->spi_t.slave_idx = slave_idx;
  sst->spi_t.select = SPISelectUnselect;
  sst->spi_t.status = SPITransDone;
  sst->spi_t.after_cb = spi_cb;

  /* Update the status and start with enabling writing */
  sst->status = SST25VFXXXX_IDLE;
  sst25vfxxxx_block_write_en(sst);
}

/**
 * Callback of the SPI after going one level higher for gathering the sst pointer
 */
void sst25vfxxxx_after_cb(struct SST25VFxxxx *sst)
{
  switch (sst->status) {
      // Enabling writing to blocks
    case SST25VFXXXX_WRITE_EN:
      // When last transmit is done
      if (sst->status_idx >= 1) {
        sst->status = SST25VFXXXX_IDLE;
        break;
      }

      // Write to the status register
      sst->status_idx = 1;
      sst->output_buf[0] = SST25VFXXXX_WRSR;
      sst->output_buf[1] = 0x0;
      sst->spi_t.output_length = 2;
      sst->spi_t.input_length = 0;
      spi_submit(sst->spi_p, &sst->spi_t);
      break;

      // Erase the full chip
    case SST25VFXXXX_CHIP_ERASE:
      switch (sst->status_idx) {
          // Execute full erase command
        case 0:
          sst->status_idx = 1;
          sst->output_buf[0] = SST25VFXXXX_ERASE_CHIP;
          sst->spi_t.output_length = 1;
          sst->spi_t.input_length = 0;
          spi_submit(sst->spi_p, &sst->spi_t);
          break;

          // Wait for chip to finish
        case 1:
          // Disable writing when finished erasing
          if (sst->spi_t.input_length == 2 && !(sst->input_buf[1] & 0x1)) {
            sst->status_idx = 2;
            sst->output_buf[0] = SST25VFXXXX_WRDI;
            sst->spi_t.output_length = 1;
            sst->spi_t.input_length = 0;
            spi_submit(sst->spi_p, &sst->spi_t);
            break;
          }
          sst->status_idx = 1;
          sst->output_buf[0] = SST25VFXXXX_RDSR;
          sst->spi_t.output_length = 1;
          sst->spi_t.input_length = 2;
          spi_submit(sst->spi_p, &sst->spi_t);
          break;

          // Goto idle
        default:
          sst->status = SST25VFXXXX_IDLE;
          break;
      }
      break;

      // Write bytes to flash
    case SST25VFXXXX_WRITE_BYTES:
      switch (sst->status_idx) {
          // Send the address with 2 or 1 byte(s) of data
        case 0:
          sst->status_idx = 1;

          if ((sst->transfer_length - sst->transfer_idx) > 1) {
            sst->output_buf[0] = SST25VFXXXX_AAI_PROG;
          } else {
            sst->output_buf[0] = SST25VFXXXX_BYTE_PROG;
          }
          sst->output_buf[1] = (sst->flash_addr >> 16) & 0xFF;
          sst->output_buf[2] = (sst->flash_addr >> 8) & 0xFF;
          sst->output_buf[3] = sst->flash_addr & 0xFF;
          sst->output_buf[4] = sst->transfer_buf[sst->transfer_idx++];

          if ((sst->transfer_length - sst->transfer_idx) > 1) {
            sst->output_buf[5] = sst->transfer_buf[sst->transfer_idx++];
            sst->spi_t.output_length = 6;
          } else {
            sst->spi_t.output_length = 5;
          }

          sst->spi_t.input_length = 0;
          spi_submit(sst->spi_p, &sst->spi_t);
          break;

          // Read the status register
        case 1:
          sst->status_idx = 2;
          sst->output_buf[0] = SST25VFXXXX_RDSR;
          sst->spi_t.output_length = 1;
          sst->spi_t.input_length = 2;
          spi_submit(sst->spi_p, &sst->spi_t);
          break;

          // Check the status register and send new bytes if possible
        case 2:
          // Still busy
          if (sst->input_buf[1] & 0x1) {
            sst->output_buf[0] = SST25VFXXXX_RDSR;
            sst->spi_t.output_length = 1;
            sst->spi_t.input_length = 2;
            spi_submit(sst->spi_p, &sst->spi_t);
            break;
          }

          // Write disabeling
          if ((sst->transfer_length - sst->transfer_idx) <= 0) {
            sst->status_idx = 3;
            sst->output_buf[0] = SST25VFXXXX_WRDI;
            sst->spi_t.output_length = 1;
            sst->spi_t.input_length = 0;
            spi_submit(sst->spi_p, &sst->spi_t);
            break;
          }

          // Transfer new bytes
          sst->status_idx = 1;
          sst->output_buf[0] = SST25VFXXXX_AAI_PROG;
          sst->output_buf[1] = sst->transfer_buf[sst->transfer_idx++];

          if ((sst->transfer_length - sst->transfer_idx) <= 0) {
            sst->output_buf[2] = sst->transfer_buf[sst->transfer_idx++];  //FIXME: uneven packets!!!!!
          } else {
            sst->output_buf[2] = 0x0;
          }

          sst->spi_t.output_length = 3;
          sst->spi_t.input_length = 0;
          spi_submit(sst->spi_p, &sst->spi_t);
          break;

          // Goto idle and update the flash address
        default:
          sst->flash_addr += sst->transfer_length;
          sst->status = SST25VFXXXX_IDLE;
          break;
      }
      break;

      // Read bytes from flash memory
    case SST25VFXXXX_READ_BYTES:
      // Reset the buffer pointer and goto idle
      sst->spi_t.input_buf = sst->input_buf;
      sst->status = SST25VFXXXX_IDLE;
      break;

      // Default goto idle
    default:
      sst->status = SST25VFXXXX_IDLE;
      break;
  }
}

/**
 * Read the chip identifier
 */
void sst25vfxxxx_read_id(struct SST25VFxxxx *sst)
{
  if (sst->status != SST25VFXXXX_IDLE) {
    return;
  }

  // Write the read id command to the chip
  sst->status = SST25VFXXXX_READ_ID;
  sst->output_buf[0] = SST25VFXXXX_RDID;
  sst->output_buf[1] = 0x0;
  sst->output_buf[2] = 0x0;
  sst->output_buf[3] = 0x0; //READ the MFG ID first
  sst->spi_t.output_length = 4;
  sst->spi_t.input_length = 8;
  spi_submit(sst->spi_p, &sst->spi_t);
}

/**
 * Enable block writing
 */
void sst25vfxxxx_block_write_en(struct SST25VFxxxx *sst)
{
  if (sst->status != SST25VFXXXX_IDLE) {
    return;
  }

  // Enable writing to the status register
  sst->status = SST25VFXXXX_WRITE_EN;
  sst->status_idx = 0;
  sst->output_buf[0] = SST25VFXXXX_EWSR;
  sst->spi_t.output_length = 1;
  sst->spi_t.input_length = 0;
  spi_submit(sst->spi_p, &sst->spi_t);
}

/**
 * Full chip erase
 */
void sst25vfxxxx_chip_erase(struct SST25VFxxxx *sst)
{
  if (sst->status != SST25VFXXXX_IDLE) {
    return;
  }

  // Enable writing
  sst->status = SST25VFXXXX_CHIP_ERASE;
  sst->status_idx = 0;
  sst->output_buf[0] = SST25VFXXXX_WREN;
  sst->spi_t.output_length = 1;
  sst->spi_t.input_length = 0;
  spi_submit(sst->spi_p, &sst->spi_t);
}

/**
 * Write bytes
 */
void sst25vfxxxx_write(struct SST25VFxxxx *sst, uint8_t *transfer_buffer, uint8_t transfer_length)
{
  if (sst->status != SST25VFXXXX_IDLE) {
    return;
  }

  // Set the transfer buffer
  sst->transfer_buf = transfer_buffer; // Not copied so keep buffer available!
  sst->transfer_idx = 0;
  sst->transfer_length = transfer_length;

  // Enable writing
  sst->status = SST25VFXXXX_WRITE_BYTES;
  sst->status_idx = 0;
  sst->output_buf[0] = SST25VFXXXX_WREN;
  sst->spi_t.output_length = 1;
  sst->spi_t.input_length = 0;
  spi_submit(sst->spi_p, &sst->spi_t);
}

/**
 * Read bytes
 * Need 5 more extra bytes because of SPI overhead
 */
void sst25vfxxxx_read(struct SST25VFxxxx *sst, uint8_t *transfer_buffer, uint8_t transfer_length)
{
  if (sst->status != SST25VFXXXX_IDLE) {
    return;
  }

  // Read all bytes at once
  sst->status = SST25VFXXXX_READ_BYTES;
  sst->status_idx = 0;
  sst->output_buf[0] = SST25VFXXXX_HGIH_SPEAD_READ;
  sst->output_buf[1] = (sst->flash_addr >> 16) & 0xFF;
  sst->output_buf[2] = (sst->flash_addr >> 8) & 0xFF;
  sst->output_buf[3] = sst->flash_addr & 0xFF;
  sst->output_buf[4] = 0x0;
  sst->spi_t.output_length = 5;
  sst->spi_t.input_buf = transfer_buffer; // Need to reset this afterwards
  sst->spi_t.input_length = transfer_length + 5;
  spi_submit(sst->spi_p, &sst->spi_t);
}
