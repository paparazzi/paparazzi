/*
 * Copyright (C) 2014 Gautier Hattenberger
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file peripherals/eeprom25AA256.h
 *
 * Driver for the eeprom 225AA256 (and 25LC256)
 * 256K SPI bus serial EEPROM from Microchip
 *
 * Currently, only reading is implemented
 */

#include "peripherals/eeprom25AA256.h"

/** Instruction set
 */
typedef enum {
  EEPROM_READ   = 0x3,
  EEPROM_WRITE  = 0x2,
  EEPROM_WRDI   = 0x4,
  EEPROM_WREN   = 0x6,
  EEPROM_RDSR   = 0x5,
  EEPROM_WRSR   = 0x1
} Eeprom25AA256InstructionSet;

// Init function
void eeprom25AA256_init(struct Eeprom25AA256 *eeprom, struct spi_periph *spi_p, uint8_t slave_idx)
{
  /* set spi_peripheral */
  eeprom->spi_p = spi_p;

  /* configure spi transaction */
  eeprom->spi_trans.cpol = SPICpolIdleLow;
  eeprom->spi_trans.cpha = SPICphaEdge1;
  eeprom->spi_trans.dss = SPIDss8bit;
  eeprom->spi_trans.bitorder = SPIMSBFirst;
  eeprom->spi_trans.cdiv = SPIDiv128; // f_PCLK / div

  eeprom->spi_trans.select = SPISelectUnselect;
  eeprom->spi_trans.slave_idx = slave_idx;
  eeprom->spi_trans.output_length = 0;
  eeprom->spi_trans.input_length = 0;
  eeprom->spi_trans.before_cb = NULL;
  eeprom->spi_trans.after_cb = NULL;
  eeprom->spi_trans.input_buf = &(eeprom->rx_buf[0]);
  eeprom->spi_trans.output_buf = &(eeprom->tx_buf[0]);

  /* set inital status: Success or Done */
  eeprom->spi_trans.status = SPITransDone;

}

// Read data
void eeprom25AA256_read(struct Eeprom25AA256 *eeprom, uint16_t addr, uint16_t length)
{
  if (E25_OUT_BUFFER_LEN < 3 || E25_IN_BUFFER_LEN < (length + 3)) {
    return; // Buffer or too small
  }
  eeprom->tx_buf[0] = EEPROM_READ;
  eeprom->tx_buf[1] = (addr >> 8) & 0xff;
  eeprom->tx_buf[2] = addr & 0xff;

  if (eeprom->spi_trans.status == SPITransDone) {
    eeprom->spi_trans.output_length = 3;
    eeprom->spi_trans.input_length = length + 3;
    spi_submit(eeprom->spi_p, &(eeprom->spi_trans));
  }
}

// Check end of transaction
void eeprom25AA256_event(struct Eeprom25AA256 *eeprom)
{
  // TODO better report status
  if (eeprom->spi_trans.status == SPITransFailed) {
    // Fail reading
    eeprom->spi_trans.status = SPITransDone;
  } else if (eeprom->spi_trans.status == SPITransSuccess) {
    // Successfull reading
    eeprom->data_available = true;
    eeprom->spi_trans.status = SPITransDone;
  }
}

