/**
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

/**
 * @file modules/sensors/ezcurrent.c
 * Implementation of driver for the EzOSD Current sensor.
 *
 * Notes:
 * Connect directly to I2C1 port.
 *
 * Sensor module wire assignments:
 * Red wire: 5V
 * Black wire: Ground
 * DAT: SDA
 * CLK: SCL
 */

#include "sensors/ezcurrent.h"
#include "mcu_periph/i2c.h"
#include "subsystems/electrical.h"

#define EZCURRENT_ADDR 0xEF

#ifndef EZCURRENT_I2C_DEV
#define EZCURRENT_I2C_DEV i2c1
#endif

struct i2c_transaction ezcurrent_i2c_trans;

void ezcurrent_init( void ) {
  electrical.vsupply = 0;
  electrical.current = 0;

  ezcurrent_i2c_trans.status = I2CTransDone;
  ezcurrent_i2c_trans.slave_addr = EZCURRENT_ADDR;
}

void ezcurrent_read_periodic( void ) {
#ifndef SITL
  if (ezcurrent_i2c_trans.status == I2CTransDone) {
    I2CReceive(EZCURRENT_I2C_DEV, ezcurrent_i2c_trans, ezcurrent_i2c_trans.slave_addr, 10);
  }
#endif //SITL
}

void ezcurrent_read_event( void ) {
  if (ezcurrent_i2c_trans.status == I2CTransSuccess) {
    // Get electrical information from buffer
    electrical.vsupply = ((uint8_t)( (((ezcurrent_i2c_trans.buf[3]) << 8) + (ezcurrent_i2c_trans.buf[2])) * 0.01f) );
    electrical.current = ((int32_t)(ezcurrent_i2c_trans.buf[9]) << 8) + (int32_t)(ezcurrent_i2c_trans.buf[8]);
    electrical.consumed = ((int32_t)(ezcurrent_i2c_trans.buf[7]) << 8) + (int32_t)(ezcurrent_i2c_trans.buf[6]);
    // Transaction has been read
    ezcurrent_i2c_trans.status = I2CTransDone;
  } else if ( ezcurrent_i2c_trans.status == I2CTransFailed ) {
    ezcurrent_i2c_trans.status = I2CTransDone;
    // ezcurrent_i2c_trans.slave_addr++;
  }
}

