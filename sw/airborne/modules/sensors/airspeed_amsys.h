/*
 * Driver for a Amsys Differential Presure Sensor I2C
 * AMS 5812-0003-D
 * AMS 5812-0001-D
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

#ifndef AIRSPEED_AMSYS_H
#define AIRSPEED_AMSYS_H

#include "std.h"
#include "mcu_periph/i2c.h"

extern float airspeed_scale;
extern float airspeed_filter;

extern struct i2c_transaction airspeed_amsys_i2c_trans;

extern void airspeed_amsys_init(void);
extern void airspeed_amsys_read_periodic(void);
extern void airspeed_amsys_read_event(void);

#define AirspeedAmsysEvent() { if (airspeed_amsys_i2c_trans.status == I2CTransSuccess) airspeed_amsys_read_event(); }

#endif // AIRSPEED_AMSYS_H
