/*
 * Copyright (C) 2015 Xavier Paris, Gautier Hattenberger
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file peripherals/ak8975.c
 *
 * Driver for the AKM AK8975 magnetometer.
 *
 */

#include "peripherals/ak8975.h"
#include "mcu_periph/sys_time.h"

#define AK8975_MEAS_TIME_MS 9

// Internal calibration coeff
// Currently fetched at startup but not used after
// Only relying on general IMU mag calibration
static float calibration_values[3] = { 0, 0, 0 };

static float __attribute__((unused)) get_ajusted_value(const int16_t val, const uint8_t axis)
{
  const float H = (float) val;
  const float corr_factor = calibration_values[axis];
  const float Hadj = corr_factor * H;

  return Hadj;
}

void ak8975_init(struct Ak8975 *ak, struct i2c_periph *i2c_p, uint8_t addr)
{
  /* set i2c_peripheral */
  ak->i2c_p = i2c_p;
  /* set i2c address */
  ak->i2c_trans.slave_addr = addr;

  ak->i2c_trans.status = I2CTransDone;

  ak->initialized = false;
  ak->status = AK_STATUS_IDLE;
  ak->init_status = AK_CONF_UNINIT;
  ak->data_available = false;
}

// Configure
void ak8975_configure(struct Ak8975 *ak)
{
  // Only configure when not busy
  if (ak->i2c_trans.status != I2CTransSuccess && ak->i2c_trans.status != I2CTransFailed
      && ak->i2c_trans.status != I2CTransDone) {
    return;
  }

  // Only when succesfull continue with next
  if (ak->i2c_trans.status == I2CTransSuccess) {
    ak->init_status++;
  }

  ak->i2c_trans.status = I2CTransDone;
  switch (ak->init_status) {

    case AK_CONF_UNINIT:
      // Set AK8975 in fuse ROM access mode to read ADC calibration data
      ak->i2c_trans.buf[0] = AK8975_REG_CNTL_ADDR;
      ak->i2c_trans.buf[1] = AK8975_MODE_FUSE_ACCESS;
      i2c_transmit(ak->i2c_p, &(ak->i2c_trans), ak->i2c_trans.slave_addr, 2);
      break;

    case AK_REQ_CALIBRATION:
      // Request AK8975 for ADC calibration data
      ak->i2c_trans.buf[0] = AK8975_REG_ASASX;
      i2c_transceive(ak->i2c_p, &(ak->i2c_trans), ak->i2c_trans.slave_addr, 1, 3);
      break;

    case AK_DISABLE_ACCESS_CALIBRATION:
      // Read config
      for (uint8_t i =0; i<=2; i++)
        calibration_values[i] =
          ((((float)(ak->i2c_trans.buf[i])  - 128.0f)*0.5f)/128.0f)+1.0f;

      // Set AK8975 in power-down mode to stop read calibration data
      ak->i2c_trans.buf[0] = AK8975_REG_CNTL_ADDR;
      ak->i2c_trans.buf[1] = AK8975_MODE_POWER_DOWN;
      i2c_transmit(ak->i2c_p, &(ak->i2c_trans), ak->i2c_trans.slave_addr, 2);
      break;

    case AK_CONF_REQUESTED:
      ak->initialized = true;
      break;

    default:
      break;
  }
}

void ak8975_read(struct Ak8975 *ak)
{
  if (ak->status != AK_STATUS_IDLE) {
    return;
  }

  // Send single measurement request
  ak->i2c_trans.buf[0] = AK8975_REG_CNTL_ADDR;
  ak->i2c_trans.buf[1] = AK8975_MODE_SINGLE_MEAS;
  i2c_transmit(ak->i2c_p, &(ak->i2c_trans), ak->i2c_trans.slave_addr, 2);
  ak->last_meas_time = get_sys_time_msec();
  ak->status = AK_STATUS_MEAS;
}

// Get raw value
#define RawFromBuf(_buf,_idx) ((int16_t)(_buf[_idx] | (_buf[_idx+1] << 8)))
// Raw is actually a 14 bits signed value
#define Int16FromRaw(_raw) ( (_raw & 0x1FFF) > 0xFFF ? (_raw & 0x1FFF) - 0x2000 : (_raw & 0x0FFF) )
void ak8975_event(struct Ak8975 *ak)
{
  if (!ak->initialized) {
    return;
  }

  switch (ak->status) {

    case AK_STATUS_MEAS:
      // Send a read data command if measurement time is done (9ms max)
      if (ak->i2c_trans.status == I2CTransSuccess &&
          (get_sys_time_msec() - ak->last_meas_time >= AK8975_MEAS_TIME_MS)) {
        ak->i2c_trans.buf[0] = AK8975_REG_ST1_ADDR;
        i2c_transceive(ak->i2c_p, &(ak->i2c_trans), ak->i2c_trans.slave_addr, 1, 8);
        ak->status++;
      }
      break;

    case AK_STATUS_READ:
      if (ak->i2c_trans.status == I2CTransSuccess) {
        // Mag data :
        //   Status 1
        //     1 byte
        //   Measures :
        //     2 bytes
        //     2 bytes
        //     2 bytes
        //   Status 2
        //     1 byte

        // Read status and error bytes
        const bool dr = ak->i2c_trans.buf[0] & 0x01; // data ready
        const bool de = ak->i2c_trans.buf[7] & 0x04; // data error
        const bool mo = ak->i2c_trans.buf[7] & 0x08; // mag overflow
        if (de || !dr) {
          // read error or data not ready, keep reading
          break;
        }
        if (mo) {
          // overflow, back to idle
          ak->status = AK_STATUS_IDLE;
        }
        // Copy the data
        int16_t val;
        val = RawFromBuf(ak->i2c_trans.buf, 1);
        ak->data.vect.x = Int16FromRaw(val);
        val = RawFromBuf(ak->i2c_trans.buf, 3);
        ak->data.vect.y = Int16FromRaw(val);
        val = RawFromBuf(ak->i2c_trans.buf, 5);
        ak->data.vect.z = Int16FromRaw(val);
        ak->data_available = true;
        // End reading, back to idle
        ak->status = AK_STATUS_IDLE;
        break;
      }
      break;

    default:
      if (ak->i2c_trans.status == I2CTransSuccess || ak->i2c_trans.status == I2CTransFailed) {
        // Goto idle
        ak->i2c_trans.status = I2CTransDone;
        ak->status = AK_STATUS_IDLE;
      }
      break;
  }
}

