/*
 * Copyright (C) 2015 Gautier Hattenberger
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

#ifndef EEPROM25AA256_H
#define EEPROM25AA256_H

#include "std.h"
#include "mcu_periph/spi.h"

/** Input buffer length
 *
 * By default read 1024 bytes after the 3 empty bytes
 * corresponding to the read command
 */
#ifndef E25_IN_BUFFER_LEN
#define E25_IN_BUFFER_LEN (1024 + 3)
#endif

/** Output buffer length
 *
 * For read only operation, we need 1 byte for read command
 * and 2 bytes for 16-bit address
 */
#ifndef E25_OUT_BUFFER_LEN
#define E25_OUT_BUFFER_LEN 3
#endif

/** 25AA256 eeprom structure
 */
struct Eeprom25AA256 {
  struct spi_periph *spi_p;                     ///< spi peripheral
  struct spi_transaction spi_trans;             ///< spi transaction
  volatile uint8_t tx_buf[E25_OUT_BUFFER_LEN];  ///< transmit buffer
  volatile uint8_t rx_buf[E25_IN_BUFFER_LEN];   ///< receive buffer
  bool_t data_available;                        ///< data read flag
};

/** Init function
 *
 * @param[in] eeprom pointer to 25AA256 eeprom struture
 * @param[in] spi_p pointer to spi device
 * @param[in] slave_idx SPI slive index
 */
extern void eeprom25AA256_init(struct Eeprom25AA256 *eeprom, struct spi_periph *spi_p, uint8_t slave_idx);

/** Read function
 *
 * @param[in] eeprom pointer to 25AA256 eeprom struture
 * @param[in] addr 16-bit start read address
 * @param[in] length number of bytes to read
 */
extern void eeprom25AA256_read(struct Eeprom25AA256 *eeprom, uint16_t addr, uint16_t length);

/** Event function
 *
 * @param[in] eeprom pointer to 25AA256 eeprom struture
 */
extern void eeprom25AA256_event(struct Eeprom25AA256 *eeprom);

#endif // EEPROM25AA256_H

