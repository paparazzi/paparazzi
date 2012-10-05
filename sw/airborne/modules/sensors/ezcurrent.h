/*
 * Driver for the EzOSD Current sensor.
 *
 * Notes:
 * Connect directly to I2C1 port.
 *
 * Sensor module wire assignments:
 * Red wire: 5V
 * Black wire: Ground
 * DAT: SDA
 * CLK: SCL
 *
 * Copyright (C) 2012 Gerard Toonstra
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

#ifndef EZCURRENT_H
#define EZCURRENT_H

#include "std.h"
#include "mcu_periph/i2c.h"

extern struct i2c_transaction ezcurrent_i2c_trans;

extern void ezcurrent_init( void );
extern void ezcurrent_read_periodic( void );
extern void ezcurrent_read_event( void );

#endif // EZCURRENT_H
