/*
 * Copyright (C) 2013 AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
 * Utah State University, http://aggieair.usu.edu/
 *
 * Michal Podhradsky (michal.podhradsky@aggiemail.usu.edu)
 * Calvin Coopmans (c.r.coopmans@ieee.org)
 *
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
 * @file arch/chibios/mcu_periph/i2c_arch.h
 * Interface from Paparazzi I2C to ChibiOS I2C driver
 *
 * I2C configuration files are defined in the board file,
 * so the maximal architecture independence is ensured.
 */
#ifndef I2C_HW_H
#define I2C_HW_H

#if USE_I2C1
extern void i2c1_hw_init(void);
#endif /* USE_I2C1 */

#if USE_I2C2
extern void i2c2_hw_init(void);
#endif /* USE_I2C2 */

#if USE_I2C3
extern void i2c3_hw_init(void);
#endif /* USE_I2C3 */

#endif /* I2C_HW_H */
