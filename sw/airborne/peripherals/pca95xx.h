/*
 * Copyright (C) 2020 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 */

/**
 * @file peripherals/pca95xx.h
 *
 * Driver for the 8-bit I/O expander based on i2c
 */

#ifndef PCA95XX_H
#define PCA95XX_H

#include "std.h"
#include "mcu_periph/i2c.h"

#define PCA95XX_DEFAULT_ADDRESS 0x40

#define PCA95XX_INPUT_REG   (0x00)
#define PCA95XX_OUTPUT_REG  (0x01)
#define PCA95XX_POL_REG     (0x02)
#define PCA95XX_CONFIG_REG  (0x03)

#define PCA95XX_P0 (1 << 0)
#define PCA95XX_P1 (1 << 1)
#define PCA95XX_P2 (1 << 2)
#define PCA95XX_P3 (1 << 3)
#define PCA95XX_P4 (1 << 4)
#define PCA95XX_P5 (1 << 5)
#define PCA95XX_P6 (1 << 6)
#define PCA95XX_P7 (1 << 7)

#define PCA95XX_CLEAR_ALL 0x00

/** PCA95XX structure
 */
struct pca95xx {
  struct i2c_periph *i2c_p;
  struct i2c_transaction i2c_trans;
};

/** Init PCA95XX
 * @param [in] dev address to pca95xx device
 * @param [in] i2c_p addres of i2c bus
 * @param [in] addr i2c address
 */
extern void pca95xx_init(struct pca95xx *dev, struct i2c_periph *i2c_p, uint8_t addr);

/** Configure PCA95XX
 * @param [in] dev address to pca95xx device
 * @param [in] val value to write to confi register
 * @param [in] blocking true for blocking i2c transaction
 * @return false if i2c was not ready or transaction submit fails or timeout (blocking)
 */
extern bool pca95xx_configure(struct pca95xx *dev, uint8_t val, bool blocking);

/** Set output value
 * @param [in] dev address to pca95xx device
 * @param [in] mask output pins to set
 * @param [in] blocking true for blocking i2c transaction
 * @return false if i2c was not ready or transaction submit fails or timeout (blocking)
 */
extern bool pca95xx_set_output(struct pca95xx *dev, uint8_t mask, bool blocking);

/** Get input value
 * @param [in] dev address to pca95xx device
 * @param [in] mask input pins
 * @param [out] input register with mask applied
 * Note: always blocking
 * @return false if i2c was not ready or transaction submit fails or timeout (blocking)
 */
extern bool pca95xx_get_input(struct pca95xx *dev, uint8_t mask, uint8_t *result);

#endif

