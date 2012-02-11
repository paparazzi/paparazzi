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
 * Pressure Board Navarro (2010)
 *
 * by Matthieu Navarro
 *
 * combine differential and absolute pressure sensor (ETS raw sensors)
 * controlled by a dspic
 * return scaled values over I2C
 *
 */


#ifndef PRESSURE_BOARD_NAVARRO_H
#define PRESSURE_BOARD_NAVARRO_H

#include "std.h"
#include "mcu_periph/i2c.h"

extern uint16_t altitude_adc;
extern uint16_t airspeed_adc;
extern uint16_t altitude_offset;
extern uint16_t airspeed_offset;
extern float pbn_altitude, pbn_airspeed;
extern float airspeed_filter;

extern bool_t data_valid;
extern struct i2c_transaction pbn_trans;

extern void pbn_init( void );
extern void pbn_periodic( void );
extern void pbn_read_event( void );

#define PbnEvent() { if (pbn_trans.status == I2CTransSuccess) pbn_read_event(); }

#define PERIODIC_SEND_PBN(_chan) DOWNLINK_SEND_PBN(DefaultChannel, DefaultDevice,&airspeed_adc,&altitude_adc,&pbn_airspeed,&pbn_altitude,&airspeed_offset,&altitude_offset);

#endif // PRESSURE_BOARD_NAVARRO_H
