/*
 * Copyright (C) 2010 ENAC
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

/*
 * Driver for i2c infrared sensor
 */

#ifndef INFRARED_I2C_H
#define INFRARED_I2C_H

#include "std.h"
#include "airframe.h"

extern float ir_i2c_roll_neutral; /* Rad */
extern float ir_i2c_pitch_neutral; /* Rad */

extern int16_t ir_i2c_ir1; /* First horizontal channel */
extern int16_t ir_i2c_ir2; /* Second horizontal channel */
extern int16_t ir_i2c_roll;  /* averaged roll adc */
extern int16_t ir_i2c_pitch; /* averaged pitch adc */
extern int16_t ir_i2c_top;  /* averaged vertical ir adc */

extern void infrared_i2c_init( void );
extern void infrared_i2c_update( void );

#endif // INFRARED_I2C_H
