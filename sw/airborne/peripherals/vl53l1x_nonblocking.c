/*
 * Copyright (C) 2020 Tom van Dijk <tomvand@users.noreply.github.com>
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
/** @file peripherals/vl53l1x_nonblocking.c
 *  @brief Non-blocking runtime functions for the VL53L1X.
 *
 *  These functions are adapted from vl53l1x_api.c to use the non-blocking
 *  version of I2C functions.
 */

#include "vl53l1x_nonblocking.h"

#include <assert.h>

// possible GetRangeStatus return value as in api.c GetRangeStatus
static const uint8_t status_rtn[24] = { 255, 255, 255, 5, 2, 4, 1, 7, 3, 0,
                                        255, 255, 9, 13, 255, 255, 255, 255, 10, 6,
                                        255, 255, 11, 12
                                      };
// Returns true upon completion
static bool VL53L1_NonBlocking_WriteMulti(VL53L1_DEV dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
  switch (dev->nonblocking.i2c_state) {
    case 0:
      assert(2 + count <= I2C_BUF_LEN);
      dev->i2c_trans.buf[0] = (index & 0xFF00) >> 8; // MSB first
      dev->i2c_trans.buf[1] = (index & 0x00FF);
      for (uint8_t i = 0; i < count; ++i) { dev->i2c_trans.buf[i + 2] = pdata[i]; }
      i2c_transmit(dev->i2c_p, &dev->i2c_trans, dev->i2c_trans.slave_addr, 2 + count);
      dev->nonblocking.i2c_state++;
    /* Falls through. */
    case 1:
      if (dev->i2c_trans.status == I2CTransFailed) {
        dev->nonblocking.i2c_state = 0;  // Try again.
        return false;
      }
      if (dev->i2c_trans.status != I2CTransSuccess) { return false; }  // Wait for transaction to complete.
      // Transaction success
      dev->nonblocking.i2c_state = 0;
      return true;
    default: return false;
  }
}

// Returns true upon completion
static bool VL53L1_NonBlocking_ReadMulti(VL53L1_DEV dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
  switch (dev->nonblocking.i2c_state) {
    case 0:
      assert(count <= I2C_BUF_LEN);
      dev->i2c_trans.buf[0] = (index & 0xFF00) >> 8; // MSB first
      dev->i2c_trans.buf[1] = (index & 0x00FF);
      i2c_transceive(dev->i2c_p, &dev->i2c_trans, dev->i2c_trans.slave_addr, 2, count);
      dev->nonblocking.i2c_state++;
    /* Falls through. */
    case 1:
      if (dev->i2c_trans.status == I2CTransFailed) {
        dev->nonblocking.i2c_state = 0;  // Try again.
        return false;
      }
      if (dev->i2c_trans.status != I2CTransSuccess) { return false; }  // Wait for transaction to complete.
      // Transaction success
      for (uint8_t i = 0; i < count; ++i) { pdata[i] = dev->i2c_trans.buf[i]; }
      dev->nonblocking.i2c_state = 0;
      return true;
    default: return false;
  }
}

// Returns true upon completion
bool VL53L1X_NonBlocking_CheckForDataReady(VL53L1_DEV dev, uint8_t *isDataReady)
{
  uint8_t Temp;
  switch (dev->nonblocking.state) {
    case 0:
      // GetInterruptPolarity
      if (!VL53L1_NonBlocking_ReadMulti(dev, GPIO_HV_MUX__CTRL, &Temp, 1)) { return false; }
      Temp = Temp & 0x10;
      dev->nonblocking.IntPol = !(Temp >> 4);
      dev->nonblocking.state++;
    /* Falls through. */
    case 1:
      /* Read in the register to check if a new value is available */
      if (!VL53L1_NonBlocking_ReadMulti(dev, GPIO__TIO_HV_STATUS, &Temp, 1)) { return false; }
      if ((Temp & 1) == dev->nonblocking.IntPol) {
        *isDataReady = 1;
      } else {
        *isDataReady = 0;
      }
      dev->nonblocking.state = 0;
      return true;
    default: return false;
  }
}

// Returns true upon completion
bool VL53L1X_NonBlocking_GetRangeStatus(VL53L1_DEV dev, uint8_t *rangeStatus)
{
  uint8_t RgSt;
  *rangeStatus = 255;
  if (!VL53L1_NonBlocking_ReadMulti(dev, VL53L1_RESULT__RANGE_STATUS, &RgSt, 1)) { return false; }
  RgSt = RgSt & 0x1F;
  if (RgSt < 24) {
    *rangeStatus = status_rtn[RgSt];
  }
  return true;
}

// Returns true upon completion
bool VL53L1X_NonBlocking_GetDistance(VL53L1_DEV dev, uint16_t *distance)
{
  uint8_t tmp[2];
  if (!VL53L1_NonBlocking_ReadMulti(dev, VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0,
                                    tmp, 2)) { return false; }
  *distance = (tmp[0] << 8) | tmp[1];
  return true;
}

// Returns true upon completion
bool VL53L1X_NonBlocking_ClearInterrupt(VL53L1_DEV dev)
{
  uint8_t data = 0x01;
  return VL53L1_NonBlocking_WriteMulti(dev, SYSTEM__INTERRUPT_CLEAR, &data, 1);
}

bool VL53L1X_NonBlocking_ReadDataEvent(VL53L1_DEV dev, uint16_t *distance_mm, bool *new_data)
{
  uint8_t isDataReady, rangeStatus;
  *new_data = false;
  switch (dev->read_status) {
    case VL53L1_READ_IDLE:
      // Idle, do nothing
      return false;
    case VL53L1_READ_DATA_READY:
      // Wait for data ready
      if (!VL53L1X_NonBlocking_CheckForDataReady(dev, &isDataReady)) {
        return false; // Check in progress
      }
      dev->read_status++;
      /* Falls through. */
    case VL53L1_READ_STATUS:
      // Get range status
      if (!VL53L1X_NonBlocking_GetRangeStatus(dev, &rangeStatus)) {
        return false; // Read status in progress
      }
      if (rangeStatus != 0) {
        dev->read_status = VL53L1_CLEAR_INT; // wrong measurement, clear interrupt and back to idle
        return true; // end cycle
      }
      dev->read_status++;
      /* Falls through. */
    case VL53L1_READ_DISTANCE:
      // Get ranging data
      if (!VL53L1X_NonBlocking_GetDistance(dev, distance_mm)) {
        return false; // Read in progress
      }
      *new_data = true;
      dev->read_status++;
      /* Falls through. */
    case VL53L1_CLEAR_INT:
      // Clear interrupt
      if (!VL53L1X_NonBlocking_ClearInterrupt(dev)) {
        return false; // Clear in progress
      }
      dev->read_status = VL53L1_READ_IDLE;
      return true; // Cycle is done
    default:
      return false;
  }
}

bool VL53L1X_NonBlocking_IsIdle(VL53L1_DEV dev)
{
  return dev->read_status == VL53L1_READ_IDLE;
}

bool VL53L1X_NonBlocking_RequestData(VL53L1_DEV dev)
{
  dev->read_status = VL53L1_READ_DATA_READY;
  return true;
}
