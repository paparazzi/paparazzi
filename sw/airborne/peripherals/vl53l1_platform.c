
/*
* This file is part of VL53L1 Platform
*
* Copyright (c) 2016, STMicroelectronics - All Rights Reserved
*
* License terms: BSD 3-clause "New" or "Revised" License.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#include "vl53l1_platform.h"
#include <string.h>
#include <time.h>
#include <math.h>

#include <assert.h>

int8_t VL53L1_WriteMulti(VL53L1_DEV dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
  assert(2 + count <= I2C_BUF_LEN);
  dev->i2c_trans.buf[0] = (index & 0xFF00) >> 8; // MSB first
  dev->i2c_trans.buf[1] = (index & 0x00FF);
  memcpy((uint8_t *) dev->i2c_trans.buf + 2, pdata, count);
  return !i2c_blocking_transmit(dev->i2c_p, &dev->i2c_trans,
                                dev->i2c_trans.slave_addr, 2 + count);
}

int8_t VL53L1_ReadMulti(VL53L1_DEV dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
  assert(count <= I2C_BUF_LEN);
  dev->i2c_trans.buf[0] = (index & 0xFF00) >> 8; // MSB first
  dev->i2c_trans.buf[1] = (index & 0x00FF);
  int8_t ret = !i2c_blocking_transceive(dev->i2c_p, &dev->i2c_trans,
                                        dev->i2c_trans.slave_addr, 2, count);
  memcpy(pdata, (uint8_t *) dev->i2c_trans.buf, count);
  return ret;
}

int8_t VL53L1_WrByte(VL53L1_DEV dev, uint16_t index, uint8_t data)
{
  return VL53L1_WriteMulti(dev, index, &data, 1);
}

int8_t VL53L1_WrWord(VL53L1_DEV dev, uint16_t index, uint16_t data)
{
  uint8_t data_u8[] = {
    (data & 0xFF00) >> 8,
    (data & 0x00FF)
  };
  return VL53L1_WriteMulti(dev, index, data_u8, 2);
}

int8_t VL53L1_WrDWord(VL53L1_DEV dev, uint16_t index, uint32_t data)
{
  uint8_t data_u8[] = {
    (data & 0xFF000000) >> 24,
    (data & 0x00FF0000) >> 16,
    (data & 0x0000FF00) >> 8,
    (data & 0x000000FF)
  };
  return VL53L1_WriteMulti(dev, index, data_u8, 4);
}

int8_t VL53L1_RdByte(VL53L1_DEV dev, uint16_t index, uint8_t *data)
{
  return VL53L1_ReadMulti(dev, index, data, 1);
}

int8_t VL53L1_RdWord(VL53L1_DEV dev, uint16_t index, uint16_t *data)
{
  uint8_t data_u8[2];
  int8_t ret = VL53L1_ReadMulti(dev, index, data_u8, 2);
  *data = (data_u8[0] << 8) | data_u8[1];
  return ret;
}

int8_t VL53L1_RdDWord(VL53L1_DEV dev, uint16_t index, uint32_t *data)
{
  uint8_t data_u8[4];
  int8_t ret = VL53L1_ReadMulti(dev, index, data_u8, 4);
  *data = (data_u8[0] << 24) |
          (data_u8[1] << 16) |
          (data_u8[2] << 8) |
          (data_u8[3]);
  return ret;
}
