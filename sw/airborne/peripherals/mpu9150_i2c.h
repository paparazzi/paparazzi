/*
 * Copyright (C) 2014 Xavier Paris
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
 * @file peripherals/mpu9150.h
 *
 * Driver for the MPU-9150 using I2C.
 */

#ifndef MPU9150_I2C_H
#define MPU9150_I2C_H

#include "peripherals/mpu60x0_i2c.h"

#ifdef MPU9150_SLV_MAG
#include "peripherals/ak8975.h"
#define MPU_MAG_SLV_NB    0
extern bool_t mpu9150_i2c_mag_event(struct Mpu60x0_I2c *mpu, struct Int32Vect3 *mag);
extern bool_t mpu9150_i2c_configure_mag_slave(Mpu60x0ConfigSet mpu_set __attribute__ ((unused)), void* mpu __attribute__  ((unused)));
#endif

#ifdef MPU9150_SLV_BARO
#include "peripherals/mpl3115.h"
#define MPU_BARO_SLV_NB   1
extern void mpu9150_i2c_baro_event(struct Mpu60x0_I2c *mpu, float *pressure);
extern bool_t mpu9150_i2c_configure_baro_slave(Mpu60x0ConfigSet mpu_set __attribute__ ((unused)), void* mpu __attribute__  ((unused)));
#endif

#endif // MPU9150_I2C_H
