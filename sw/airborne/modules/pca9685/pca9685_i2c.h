/*
 * Copyright (C) 2013-2020 Chris Efstathiou hendrixgr@gmail.com
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
 * @file modules/pca9685/pca9685.h
 * PCA9685 LED DRIVER USED AS A 16 ADDITIONAL PWM SERVO DRIVER.
 *
 */


#ifndef PCA9685_I2C_H
#define PCA9685_I2C_H

#include "std.h"

#define PCA9865_SRV0  PCA9685_LED0_OFF_L_REG_ADDR
#define PCA9865_SRV1  PCA9685_LED1_OFF_L_REG_ADDR
#define PCA9865_SRV2  PCA9685_LED2_OFF_L_REG_ADDR
#define PCA9865_SRV3  PCA9685_LED3_OFF_L_REG_ADDR
#define PCA9865_SRV4  PCA9685_LED4_OFF_L_REG_ADDR
#define PCA9865_SRV5  PCA9685_LED5_OFF_L_REG_ADDR
#define PCA9865_SRV6  PCA9685_LED6_OFF_L_REG_ADDR
#define PCA9865_SRV7  PCA9685_LED7_OFF_L_REG_ADDR
#define PCA9865_SRV8  PCA9685_LED8_OFF_L_REG_ADDR
#define PCA9865_SRV9  PCA9685_LED9_OFF_L_REG_ADDR
#define PCA9865_SRV10 PCA9685_LED10_OFF_L_REG_ADDR
#define PCA9865_SRV11 PCA9685_LED11_OFF_L_REG_ADDR
#define PCA9865_SRV12 PCA9685_LED12_OFF_L_REG_ADDR
#define PCA9865_SRV13 PCA9685_LED13_OFF_L_REG_ADDR
#define PCA9865_SRV14 PCA9685_LED14_OFF_L_REG_ADDR
#define PCA9865_SRV15 PCA9685_LED15_OFF_L_REG_ADDR

extern void pca9685_i2c_init(void);
extern void pca9685_i2c_periodic(void);
extern void pca9685_i2c_event(void);

bool pca9865_set_servo(uint8_t srv_nb, uint16_t srv_val);

#endif
