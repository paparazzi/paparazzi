/*
 * Driver for a Amsys Barometric Sensor I2C
 * AMS 5812-0150-A
 *
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

#ifndef BARO_AMSYS_H
#define BARO_AMSYS_H

#include "std.h"
#include "mcu_periph/i2c.h"

/// new measurement every baro_amsys_read_periodic
#define BARO_AMSYS_DT BARO_AMSYS_READ_PERIODIC_PERIOD

extern uint16_t baro_amsys_adc;
// extern float baro_amsys_offset;
extern bool baro_amsys_valid;
extern bool baro_amsys_enabled;
extern float baro_amsys_altitude;
extern float baro_amsys_r;
extern float baro_amsys_sigma2;
extern float baro_filter;

extern struct i2c_transaction baro_amsys_i2c_trans;

extern void baro_amsys_init(void);
extern void baro_amsys_read_periodic(void);
extern void baro_amsys_read_event(void);

#define BaroAmsysEvent() { if (baro_amsys_i2c_trans.status == I2CTransSuccess) baro_amsys_read_event(); }

#endif // BARO_AMSYS_H
