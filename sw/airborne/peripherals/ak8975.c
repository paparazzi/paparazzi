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
 * @file peripherals/ak8975.c
 *
 */

#include "peripherals/ak8975.h"

static float calibrationValue[3]={0,0,0};
float  i2cMasterGetAK8975_ajustedValue(const int16_t rawValue, const uint8_t axis)
{
  const float H = (float) rawValue;
  const float CorrFactor = calibrationValue[axis];
  const float Hadj = CorrFactor * H;

  return Hadj;
}

void ak8975_init(struct Ak8975 *ak, struct i2c_periph *i2c_p, uint8_t addr)
{
  /* set i2c_peripheral */
  ak->i2c_p = i2c_p;
  /* set i2c address */
  ak->i2c_trans.slave_addr = addr;

  ak->i2c_trans.status = I2CTransDone;

  ak->initialized = FALSE;
  ak->init_status = AK_CONF_UNINIT;
}

static void ak8975_request_config(struct Ak8975 *ak)
{
  switch (ak->init_status) {

    case AK_CONF_UNINIT:
      // Set AK8975 in fuse ROM access mode to read ADC calibration data
      ak->i2c_trans.buf[0] = AK8975_REG_CNTL_ADDR;
      ak->i2c_trans.buf[1] = AK8975_MODE_FUSE_ACCESS;
      i2c_transmit(ak->i2c_p, &(ak->i2c_trans), ak->i2c_trans.slave_addr, 2);
      ak->init_status++;
      break; 

    case AK_REQ_CALIBRATION:
      // Request AK8975 for ADC calibration data 
      ak->i2c_trans.buf[0] = AK8975_REG_ASASX;
      i2c_transceive(ak->i2c_p, &(ak->i2c_trans), ak->i2c_trans.slave_addr, 1, 3);
      ak->init_status++;
      break;

    case AK_DISABLE_ACCESS_CALIBRATION:
      // Read config
      for (uint8_t i =0; i<=2; i++)
        calibrationValue[i] = 
          ((((float)(ak->i2c_trans.buf[i])  - 128.0f)*0.5f)/128.0f)+1.0f;

      // Set AK8975 in power-down mode to stop read calibration data
      ak->i2c_trans.buf[0] = AK8975_REG_CNTL_ADDR;
      ak->i2c_trans.buf[1] = AK8975_MODE_POWER_DOWN;
      i2c_transmit(ak->i2c_p, &(ak->i2c_trans), ak->i2c_trans.slave_addr, 2);
      ak->init_status++;
      break;

    case AK_CONF_REQUESTED:
      ak->initialized = TRUE;
      ak->i2c_trans.status = I2CTransDone;
      break;

    default:
      break;
  }
}

// Configure
bool_t ak8975_mpu_configure(struct Ak8975 *ak)
{
  if (ak->initialized == FALSE) {
    if (ak->i2c_trans.status == I2CTransSuccess || ak->i2c_trans.status == I2CTransDone) {
      ak8975_request_config(ak);
    }
  }
  return (ak->initialized);
}
