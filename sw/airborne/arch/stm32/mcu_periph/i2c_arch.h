/*
 * $Id$
 *
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
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

/*
 * Hardware level I2C handling
 */

#ifndef I2C_HW_H
#define I2C_HW_H

#include "mcu_periph/i2c.h"
#include <libopencm3/stm32/i2c.h>

#ifdef USE_I2C1

extern void i2c1_hw_init(void);
extern void i2c1_ev_irq_handler(void);
extern void i2c1_er_irq_handler(void);

#endif /* USE_I2C1 */



#ifdef USE_I2C2

extern void i2c2_hw_init(void);
extern void i2c2_ev_irq_handler(void);
extern void i2c2_er_irq_handler(void);


#endif /* USE_I2C2 */


#endif /* I2C_HW_H */
