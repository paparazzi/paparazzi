/*
 * Copyright (C) 2020 Tom van Dijk <tomvand@users.noreply.github.com>
 * Adapted from STSW-IMG009 vl53l1_platform.h
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
 * @file  vl53l1_platform.h
 * @brief Those platform functions are platform dependent and have to be implemented by the user
 */

#ifndef _VL53L1_PLATFORM_H_
#define _VL53L1_PLATFORM_H_

#include "vl53l1_types.h"

#include "mcu_periph/i2c.h"

#ifdef __cplusplus
extern "C"
{
#endif



typedef struct {
  struct i2c_periph *i2c_p;
  struct i2c_transaction i2c_trans;
  struct NonBlocking {
    /* I2C Read/write state machine */
    uint8_t i2c_state;
    /* State machine for nonblocking functions */
    uint8_t state;
    uint8_t IntPol;
  } nonblocking;
} VL53L1_Dev_t;

typedef VL53L1_Dev_t *VL53L1_DEV;

/** @brief VL53L1_WriteMulti() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_WriteMulti(
  VL53L1_DEV      dev,
  uint16_t      index,
  uint8_t      *pdata,
  uint32_t      count);
/** @brief VL53L1_ReadMulti() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_ReadMulti(
  VL53L1_DEV      dev,
  uint16_t      index,
  uint8_t      *pdata,
  uint32_t      count);
/** @brief VL53L1_WrByte() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_WrByte(
  VL53L1_DEV dev,
  uint16_t      index,
  uint8_t       data);
/** @brief VL53L1_WrWord() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_WrWord(
  VL53L1_DEV dev,
  uint16_t      index,
  uint16_t      data);
/** @brief VL53L1_WrDWord() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_WrDWord(
  VL53L1_DEV dev,
  uint16_t      index,
  uint32_t      data);
/** @brief VL53L1_RdByte() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_RdByte(
  VL53L1_DEV dev,
  uint16_t      index,
  uint8_t      *pdata);
/** @brief VL53L1_RdWord() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_RdWord(
  VL53L1_DEV dev,
  uint16_t      index,
  uint16_t     *pdata);
/** @brief VL53L1_RdDWord() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_RdDWord(
  VL53L1_DEV dev,
  uint16_t      index,
  uint32_t     *pdata);


#ifdef __cplusplus
}
#endif

#endif
