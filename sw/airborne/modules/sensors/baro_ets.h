/*
 * Copyright (C) 2009 Vassilis Varveropoulos
 * Copyright (C) 2010 The Paparazzi Team
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
 * @file baro_ets.h
 *
 * Driver for the EagleTree Systems Altitude Sensor.
 * Has only been tested with V3 of the sensor hardware.
 *
 * Notes:
 * Connect directly to TWOG/Tiny I2C port. Multiple sensors can be chained together.
 * Sensor should be in the proprietary mode (default) and not in 3rd party mode.
 * Pitch gains may need to be updated.
 *
 *
 * Sensor module wire assignments:
 * Red wire: 5V
 * White wire: Ground
 * Yellow wire: SDA
 * Brown wire: SCL
 */

#ifndef BARO_ETS_H
#define BARO_ETS_H

#include "std.h"
#include "mcu_periph/i2c.h"

/// new measurement every baro_ets_read_periodic
#define BARO_ETS_DT BARO_ETS_READ_PERIODIC_PERIOD

extern uint16_t baro_ets_adc;
extern uint16_t baro_ets_offset;
extern bool_t baro_ets_valid;
extern bool_t baro_ets_enabled;
extern float baro_ets_altitude;
extern float baro_ets_r;
extern float baro_ets_sigma2;

extern struct i2c_transaction baro_ets_i2c_trans;

extern void baro_ets_init(void);
extern void baro_ets_read_periodic(void);
extern void baro_ets_read_event(void);

#define BaroEtsEvent() { if (baro_ets_i2c_trans.status == I2CTransSuccess) baro_ets_read_event(); }

#endif // BARO_ETS_H
