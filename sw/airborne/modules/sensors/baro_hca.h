/*
 * Copyright (C) 2012 Gautier Hattenberger (ENAC)
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

#ifndef BARO_HCA_H
#define BARO_HCA_H

#include "std.h"
#include "mcu_periph/i2c.h"


extern struct i2c_transaction baro_hca_i2c_trans;

extern void baro_hca_init(void);
extern void baro_hca_read_periodic(void);
extern void baro_hca_read_event(void);

#define BaroHcaEvent() { \
    if (baro_hca_i2c_trans.status == I2CTransSuccess) baro_hca_read_event(); \
    else if (baro_hca_i2c_trans.status == I2CTransFailed) baro_hca_i2c_trans.status = I2CTransDone; \
  }

#endif
