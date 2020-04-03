/*
 * Copyright (C) 2020 Tom van Dijk <tomvand@users.noreply.github.com>
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
 * @file mcu_periph/softi2c.h
 * Platform-independent software I2C implementation.
 * Can be used transparently in place of the hardware I2C in i2c.h.
 */

#ifndef MCU_PERIPH_SOFTI2C_H
#define MCU_PERIPH_SOFTI2C_H

#include "mcu_periph/i2c.h"


void softi2c_event(void);


#if USE_SOFTI2C0
extern void softi2c0_hw_init(void);
#endif /* USE_SOFTI2C0 */

#if USE_SOFTI2C1
extern void softi2c1_hw_init(void);
#endif /* USE_SOFTI2C1 */


#endif // MCU_PERIPH_SOFTI2C_H
