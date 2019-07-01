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

void ezcurrent_init(void)
{
  ezcurrent_i2c_trans.status = I2CTransDone;
  ezcurrent_i2c_trans.slave_addr = EZCURRENT_ADDR;
}

void ezcurrent_read_periodic(void)
{
  if (ezcurrent_i2c_trans.status == I2CTransDone) {
    i2c_receive(&EZCURRENT_I2C_DEV, &ezcurrent_i2c_trans, ezcurrent_i2c_trans.slave_addr, 10);
  }
}

#define Uint16FromBuf(_buf,_idx) ((uint16_t)((_buf[_idx+1]<<8) | _buf[_idx]))
#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx+1]<<8) | _buf[_idx]))

void ezcurrent_read_event(void)
{
  if (ezcurrent_i2c_trans.status == I2CTransSuccess) {
    /* voltage of EzOSD sensor is provided in mV */
    electrical.vsupply = (float)(Uint16FromBuf(ezcurrent_i2c_trans.buf, 2)) / 1000.f;
    /* consumed electric charge in mAh */
    electrical.charge = (float)(Int16FromBuf(ezcurrent_i2c_trans.buf, 6)) / 1000.f;
    /* sensor provides current in 1e-1 Ampere */
    electrical.current = (float)(Int16FromBuf(ezcurrent_i2c_trans.buf, 8)) / 10.f;

    // Transaction has been read
    ezcurrent_i2c_trans.status = I2CTransDone;
  } else if (ezcurrent_i2c_trans.status == I2CTransFailed) {
    ezcurrent_i2c_trans.status = I2CTransDone;
    // ezcurrent_i2c_trans.slave_addr++;
  }
}

