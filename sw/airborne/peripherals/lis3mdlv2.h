/*
 * Copyright (C) 2019 Alexis Cornard <alexiscornard@gmail.com>
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
 * @file peripherals/lis3mdlv2.h
 *
 * LIS3MDL magnetometer driver interface.
 */

#ifndef LIS3MDL_H
#define LIS3MDL_H

#ifndef LIS3MDL_DEFAULT_ODR
#define LIS3MDL_DEFAULT_ODR (LIS3MDL_ODR_10HZ << 2)
#endif

#ifndef LIS3MDL_DEFAULT_FAST_ODR
#define LIS3MDL_DEFAULT_FAST_ODR (LIS3MDL_FODR_D << 1)
#endif

#ifndef LIS3MDL_DEFAULT_OM_XY
#define LIS3MDL_DEFAULT_OM_XY (LIS3MDL_XY_OM_UHP << 5)
#endif

#ifndef LIS3MDL_DEFAULT_FS
#define LIS3MDL_DEFAULT_FS (LIS3MDL_ODR_10HZ << 5)
#endif

#ifndef LIS3MDL_DEFAULT_OM_Z
#define LIS3MDL_DEFAULT_OM_Z (LIS3MDL_Z_OM_UHP << 2)
#endif

#ifndef MAG_SENS_H
#define MAG_SENS_H

#define LIS3MDL_MAG_SENS_4G 3.5
#define LIS3MDL_MAG_SENS_4G_NUM 7
#define LIS3MDL_MAG_SENS_4G_DEN 2
#define LIS3MDL_MAG_SENS_8G 3.5
#define LIS3MDL_MAG_SENS_8G_NUM 7
#define LIS3MDL_MAG_SENS_8G_DEN 2
#define LIS3MDL_MAG_SENS_12G 3.5
#define LIS3MDL_MAG_SENS_12G_NUM 7
#define LIS3MDL_MAG_SENS_12G_DEN 2
#define LIS3MDL_MAG_SENS_16G 3.5
#define LIS3MDL_MAG_SENS_16G_NUM 7
#define LIS3MDL_MAG_SENS_16G_DEN 2

#endif

#include "std.h"

/* Include address and register definition */
#include "peripherals/lis3mdl_regs.h"

enum Lis3mdlConfStatus {
    LIS3MDL_CONF_UNINIT,
    LIS3MDL_CONF_CTRL1,
    LIS3MDL_CONF_CTRL2,
    LIS3MDL_CONF_CTRL3,
    LIS3MDL_CONF_CTRL4,
    LIS3MDL_CONF_DONE
};

struct Lis3mdlConfig {
    uint8_t ctrl1;
    uint8_t ctrl2;
    uint8_t ctrl3;
    uint8_t ctrl4;

};

static inline void lis3mdl_set_default_config(struct Lis3mdlConfig *c)
{
  c->ctrl1 = LIS3MDL_DEFAULT_ODR | LIS3MDL_DEFAULT_FAST_ODR | LIS3MDL_DEFAULT_OM_XY;
  c->ctrl2 = LIS3MDL_DEFAULT_FS;
  c->ctrl3 = 0x00;
  c->ctrl4 = LIS3MDL_DEFAULT_OM_Z;
}


#endif // LIS3MDL_H
