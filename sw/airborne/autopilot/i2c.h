/*
 * $Id$
 *  
 * Copyright (C) 2005  Pascal Brisset, Antoine Drouin
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
/** \file i2c.h
 *  \brief Basic library for I2C
 *
 */

#ifndef I2C_H
#define I2C_H

#include <avr/io.h>
#include <inttypes.h>
#include "std.h"

#define I2C_NO_ERROR 0

#define I2C_RECEIVE	1
#define I2C_TRANSMIT	0

#define I2C_QUIT	0
#define I2C_CONTINUE	1

#define i2c_stop() TWCR=_BV(TWINT)|_BV(TWSTO)|_BV(TWEN);

#define TWI_BUF_LEN 16
extern volatile bool_t i2c_idle;
extern uint8_t i2c_debug;
extern uint8_t i2c_buf[TWI_BUF_LEN];

extern void i2c_init(void);
extern uint8_t i2c_start(void);
extern uint8_t i2c_sla(uint8_t x);
extern uint8_t i2c_transmit(uint8_t x);
extern uint8_t i2c_receive(uint8_t);
extern void i2c_send(uint8_t address, uint8_t len);
extern void i2c_get(uint8_t address, uint8_t len);

#define I2C_START(ADDRESS)     { i2c_start(); i2c_sla(ADDRESS); }
#define I2C_START_TX(ADDRESS)  I2C_START(ADDRESS | I2C_TRANSMIT)
#define I2C_START_RX(ADDRESS)  I2C_START(ADDRESS | I2C_RECEIVE)

#endif
