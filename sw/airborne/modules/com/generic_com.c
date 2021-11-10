/*
 * Copyright (C) 2011 Gautier Hattenberger
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

/* Generic module to send data to a gsm/satcom module controlled by arduino over i2c */

#include "std.h"

#include "modules/com/generic_com.h"
#include "mcu_periph/i2c.h"

#include "state.h"
#include "subsystems/gps.h"
#include "modules/energy/electrical.h"
#include "generated/airframe.h"
#include "inter_mcu.h"
#include "autopilot.h"
#include "subsystems/navigation/common_nav.h"

#define NB_DATA 24

#ifndef GENERIC_COM_I2C_DEV
#define GENERIC_COM_I2C_DEV i2c0
#endif

#ifndef GENERIC_COM_SLAVE_ADDR
#define GENERIC_COM_SLAVE_ADDR 0x26
#endif

struct i2c_transaction com_trans;

bool active_com;

void generic_com_init(void)
{
  active_com = false;
  com_trans.status = I2CTransDone;
}

#define FillBufWith32bit(_buf, _index, _value) {  \
    _buf[_index] = (uint8_t) (_value);              \
    _buf[_index+1] = (uint8_t) ((_value) >> 8);     \
    _buf[_index+2] = (uint8_t) ((_value) >> 16);    \
    _buf[_index+3] = (uint8_t) ((_value) >> 24);    \
  }

#define FillBufWith16bit(_buf, _index, _value) {  \
    _buf[_index] = (uint8_t) (_value);              \
    _buf[_index+1] = (uint8_t) ((_value) >> 8);     \
  }

void generic_com_periodic(void)
{

  if (com_trans.status != I2CTransDone) { return; }

  com_trans.buf[0] = active_com;
  FillBufWith32bit(com_trans.buf, 1, gps.lla_pos.lat);
  FillBufWith32bit(com_trans.buf, 5, gps.lla_pos.lon);
  FillBufWith16bit(com_trans.buf, 9, (int16_t)(gps.lla_pos.alt / 1000)); // altitude (meters)
  FillBufWith16bit(com_trans.buf, 11, gps.gspeed); // ground speed (cm/s)
  FillBufWith16bit(com_trans.buf, 13, (int16_t)(gps.course / 1e4)); // course (1e3rad)
  FillBufWith16bit(com_trans.buf, 15, (uint16_t)(stateGetAirspeed_f() * 100)); // TAS (cm/s)
  uint8_t vsupply = Min(electrical.vsupply * 10.f, 255); // deciVolt
  uint8_t charge  = Min(electrical.vsupply * 10.f, 255); // deciAh
  com_trans.buf[17] = vsupply;
  com_trans.buf[18] = charge;
  com_trans.buf[19] = (uint8_t)(imcu_get_command(COMMAND_THROTTLE) * 100 / MAX_PPRZ);
  com_trans.buf[20] = autopilot_get_mode();
  com_trans.buf[21] = nav_block;
  FillBufWith16bit(com_trans.buf, 22, autopilot.flight_time);
  i2c_transmit(&GENERIC_COM_I2C_DEV, &com_trans, GENERIC_COM_SLAVE_ADDR, NB_DATA);
}

void generic_com_event(void)
{
  // Handle I2C event
  if (com_trans.status == I2CTransSuccess || com_trans.status == I2CTransFailed) {
    com_trans.status = I2CTransDone;
  }
}

void start_com(void)
{
  active_com = true;
  com_trans.status = I2CTransDone;
}

void stop_com(void)
{
  active_com = false;
  com_trans.buf[0] = active_com;
  i2c_transmit(&GENERIC_COM_I2C_DEV, &com_trans, GENERIC_COM_SLAVE_ADDR, 1);
}

