/*
 * Copyright (C) 2016 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file boards/bebop/mt9f002.c
 * Initialization of MT9F002 chip and options to change settings
 */

#include "std.h"
#include "mt9f002.h"
#include "mt9f002_regs.h"

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

/* Write multiple bytes to a single register */
static void write_reg(int fd, uint16_t addr, uint32_t val, uint8_t len)
{
  uint8_t buf[len + 2];
  buf[0] = addr >> 8;
  buf[1] = addr & 0xFF;

  // Fix sigdness based on length
  if(len == 1) {
    buf[3] = val & 0xFF;
  }
  else if(len == 2) {
    buf[3] = (val >> 8) & 0xFF;
    buf[4] = val & 0xFF;
  }
  else if(len == 4) {
    buf[3] = (val >> 24) & 0xFF;
    buf[4] = (val >> 16) & 0xFF;
    buf[5] = (val >> 8) & 0xFF;
    buf[6] = val & 0xFF;
  }
  else {
    printf("[MT9F002] write_reg with incorrect length %d\r\n", len);
  }

  // Transmit the buffer
  if (write(fd, buf, len + 2) != (len + 2)) {
    printf("[MT9F002] write_reg failed (fd: %d, addr: %d, len:%d)\r\n", fd, addr, len);
  }
}

/* Read multiple bytes from a register */
static uint32_t read_reg(int fd, uint16_t addr, uint8_t len)
{
  uint32_t ret = 0;
  uint8_t val[len];
  uint8_t buf[2];
  buf[0] = addr >> 8;
  buf[1] = addr & 0xFF;

  /* First transmit the address */
  if (write(fd, buf, 2) != 2) {
    printf("[MT9F002] read_reg failed, because of writing (fd: %d, addr: %d, len:%d)\r\n", fd, addr, len);
    return ret;
  }

  /* Actually read the register */
  if (read(fd, val, len) != len) {
    printf("[MT9F002] read_reg failed, because of reading (fd: %d, addr: %d, len:%d)\r\n", fd, addr, len);
    return ret;
  }

  /* Fix sigdness */
  for(uint8_t i =0; i < len; i++) {
    ret |= val[len-i-1] << (8*i);
  }
  return ret;
}

/* Configure stage 1 for both MiPi and HiSPi connection */
static void mt9f002_mipi_stage1(int fd)
{
  write_reg(fd, MT9F002_RESET_REGISTER, 0x0118, 2);
  write_reg(fd, MT9F002_MODE_SELECT, 0x00, 1);

  uint32_t serialFormat;
  if (ctx->interface == HAL_MT9F002_HiSPi)
  {
    serialFormat = (3<<8)|ctx->numberOfLanes; // parrot flex
  }
  else
  {
    serialFormat = (2<<8)|ctx->numberOfLanes;// mipi demo board
  }
  write_reg(fd, MT9F002_SERIAL_FORMAT, serialFormat, 2);
  uint32_t dataFormat = (ctx->pixelDepth<<8)|ctx->pixelDepth;
  write_reg(fd, MT9F002_CPP_DATA_FORMAT, dataFormat, 2);

  write_reg(fd, MT9F002_MFR_3D00, 0x0435, 2);
  write_reg(fd, MT9F002_MFR_3D02, 0x435D, 2);
  write_reg(fd, MT9F002_MFR_3D04, 0x6698, 2);
  write_reg(fd, MT9F002_MFR_3D06, 0xFFFF, 2);
  write_reg(fd, MT9F002_MFR_3D08, 0x7783, 2);
  write_reg(fd, MT9F002_MFR_3D0A, 0x101B, 2);
  write_reg(fd, MT9F002_MFR_3D0C, 0x732C, 2);
  write_reg(fd, MT9F002_MFR_3D0E, 0x4230, 2);
  write_reg(fd, MT9F002_MFR_3D10, 0x5881, 2);
  write_reg(fd, MT9F002_MFR_3D12, 0x5C3A, 2);
  write_reg(fd, MT9F002_MFR_3D14, 0x0140, 2);
  write_reg(fd, MT9F002_MFR_3D16, 0x2300, 2);
  write_reg(fd, MT9F002_MFR_3D18, 0x815F, 2);
  write_reg(fd, MT9F002_MFR_3D1A, 0x6789, 2);
  write_reg(fd, MT9F002_MFR_3D1C, 0x5920, 2);
  write_reg(fd, MT9F002_MFR_3D1E, 0x0C20, 2);
  write_reg(fd, MT9F002_MFR_3D20, 0x21C0, 2);
  write_reg(fd, MT9F002_MFR_3D22, 0x4684, 2);
  write_reg(fd, MT9F002_MFR_3D24, 0x4892, 2);
  write_reg(fd, MT9F002_MFR_3D26, 0x1A00, 2);
  write_reg(fd, MT9F002_MFR_3D28, 0xBA4C, 2);
  write_reg(fd, MT9F002_MFR_3D2A, 0x8D48, 2);
  write_reg(fd, MT9F002_MFR_3D2C, 0x4641, 2);
  write_reg(fd, MT9F002_MFR_3D2E, 0x408C, 2);
  write_reg(fd, MT9F002_MFR_3D30, 0x4784, 2);
  write_reg(fd, MT9F002_MFR_3D32, 0x4A87, 2);
  write_reg(fd, MT9F002_MFR_3D34, 0x561A, 2);
  write_reg(fd, MT9F002_MFR_3D36, 0x00A5, 2);
  write_reg(fd, MT9F002_MFR_3D38, 0x1A00, 2);
  write_reg(fd, MT9F002_MFR_3D3A, 0x5693, 2);
  write_reg(fd, MT9F002_MFR_3D3C, 0x4D8D, 2);
  write_reg(fd, MT9F002_MFR_3D3E, 0x4A47, 2);
  write_reg(fd, MT9F002_MFR_3D40, 0x4041, 2);
  write_reg(fd, MT9F002_MFR_3D42, 0x8200, 2);
  write_reg(fd, MT9F002_MFR_3D44, 0x24B7, 2);
  write_reg(fd, MT9F002_MFR_3D46, 0x0024, 2);
  write_reg(fd, MT9F002_MFR_3D48, 0x8D4F, 2);
  write_reg(fd, MT9F002_MFR_3D4A, 0x831A, 2);
  write_reg(fd, MT9F002_MFR_3D4C, 0x00B4, 2);
  write_reg(fd, MT9F002_MFR_3D4E, 0x4684, 2);
  write_reg(fd, MT9F002_MFR_3D50, 0x49CE, 2);
  write_reg(fd, MT9F002_MFR_3D52, 0x4946, 2);
  write_reg(fd, MT9F002_MFR_3D54, 0x4140, 2);
  write_reg(fd, MT9F002_MFR_3D56, 0x9247, 2);
  write_reg(fd, MT9F002_MFR_3D58, 0x844B, 2);
  write_reg(fd, MT9F002_MFR_3D5A, 0xCE4B, 2);
  write_reg(fd, MT9F002_MFR_3D5C, 0x4741, 2);
  write_reg(fd, MT9F002_MFR_3D5E, 0x502F, 2);
  write_reg(fd, MT9F002_MFR_3D60, 0xBD3A, 2);
  write_reg(fd, MT9F002_MFR_3D62, 0x5181, 2);
  write_reg(fd, MT9F002_MFR_3D64, 0x5E73, 2);
  write_reg(fd, MT9F002_MFR_3D66, 0x7C0A, 2);
  write_reg(fd, MT9F002_MFR_3D68, 0x7770, 2);
  write_reg(fd, MT9F002_MFR_3D6A, 0x8085, 2);
  write_reg(fd, MT9F002_MFR_3D6C, 0x6A82, 2);
  write_reg(fd, MT9F002_MFR_3D6E, 0x6742, 2);
  write_reg(fd, MT9F002_MFR_3D70, 0x8244, 2);
  write_reg(fd, MT9F002_MFR_3D72, 0x831A, 2);
  write_reg(fd, MT9F002_MFR_3D74, 0x0099, 2);
  write_reg(fd, MT9F002_MFR_3D76, 0x44DF, 2);
  write_reg(fd, MT9F002_MFR_3D78, 0x1A00, 2);
  write_reg(fd, MT9F002_MFR_3D7A, 0x8542, 2);
  write_reg(fd, MT9F002_MFR_3D7C, 0x8567, 2);
  write_reg(fd, MT9F002_MFR_3D7E, 0x826A, 2);
  write_reg(fd, MT9F002_MFR_3D80, 0x857C, 2);
  write_reg(fd, MT9F002_MFR_3D82, 0x6B80, 2);
  write_reg(fd, MT9F002_MFR_3D84, 0x7000, 2);
  write_reg(fd, MT9F002_MFR_3D86, 0xB831, 2);
  write_reg(fd, MT9F002_MFR_3D88, 0x40BE, 2);
  write_reg(fd, MT9F002_MFR_3D8A, 0x6700, 2);
  write_reg(fd, MT9F002_MFR_3D8C, 0x0CBD, 2);
  write_reg(fd, MT9F002_MFR_3D8E, 0x4482, 2);
  write_reg(fd, MT9F002_MFR_3D90, 0x7898, 2);
  write_reg(fd, MT9F002_MFR_3D92, 0x7480, 2);
  write_reg(fd, MT9F002_MFR_3D94, 0x5680, 2);
  write_reg(fd, MT9F002_MFR_3D96, 0x9755, 2);
  write_reg(fd, MT9F002_MFR_3D98, 0x8057, 2);
  write_reg(fd, MT9F002_MFR_3D9A, 0x8056, 2);
  write_reg(fd, MT9F002_MFR_3D9C, 0x9256, 2);
  write_reg(fd, MT9F002_MFR_3D9E, 0x8057, 2);
  write_reg(fd, MT9F002_MFR_3DA0, 0x8055, 2);
  write_reg(fd, MT9F002_MFR_3DA2, 0x817C, 2);
  write_reg(fd, MT9F002_MFR_3DA4, 0x969B, 2);
  write_reg(fd, MT9F002_MFR_3DA6, 0x56A6, 2);
  write_reg(fd, MT9F002_MFR_3DA8, 0x44BE, 2);
  write_reg(fd, MT9F002_MFR_3DAA, 0x000C, 2);
  write_reg(fd, MT9F002_MFR_3DAC, 0x867A, 2);
  write_reg(fd, MT9F002_MFR_3DAE, 0x9474, 2);
  write_reg(fd, MT9F002_MFR_3DB0, 0x8A79, 2);
  write_reg(fd, MT9F002_MFR_3DB2, 0x9367, 2);
  write_reg(fd, MT9F002_MFR_3DB4, 0xBF6A, 2);
  write_reg(fd, MT9F002_MFR_3DB6, 0x816C, 2);
  write_reg(fd, MT9F002_MFR_3DB8, 0x8570, 2);
  write_reg(fd, MT9F002_MFR_3DBA, 0x836C, 2);
  write_reg(fd, MT9F002_MFR_3DBC, 0x826A, 2);
  write_reg(fd, MT9F002_MFR_3DBE, 0x8245, 2);
  write_reg(fd, MT9F002_MFR_3DC0, 0xFFFF, 2);
  write_reg(fd, MT9F002_MFR_3DC2, 0xFFD6, 2);
  write_reg(fd, MT9F002_MFR_3DC4, 0x4582, 2);
  write_reg(fd, MT9F002_MFR_3DC6, 0x6A82, 2);
  write_reg(fd, MT9F002_MFR_3DC8, 0x6C83, 2);
  write_reg(fd, MT9F002_MFR_3DCA, 0x7000, 2);
  write_reg(fd, MT9F002_MFR_3DCC, 0x8024, 2);
  write_reg(fd, MT9F002_MFR_3DCE, 0xB181, 2);
  write_reg(fd, MT9F002_MFR_3DD0, 0x6859, 2);
  write_reg(fd, MT9F002_MFR_3DD2, 0x732B, 2);
  write_reg(fd, MT9F002_MFR_3DD4, 0x4030, 2);
  write_reg(fd, MT9F002_MFR_3DD6, 0x4982, 2);
  write_reg(fd, MT9F002_MFR_3DD8, 0x101B, 2);
  write_reg(fd, MT9F002_MFR_3DDA, 0x4083, 2);
  write_reg(fd, MT9F002_MFR_3DDC, 0x6785, 2);
  write_reg(fd, MT9F002_MFR_3DDE, 0x3A00, 2);
  write_reg(fd, MT9F002_MFR_3DE0, 0x8820, 2);
  write_reg(fd, MT9F002_MFR_3DE2, 0x0C59, 2);
  write_reg(fd, MT9F002_MFR_3DE4, 0x8546, 2);
  write_reg(fd, MT9F002_MFR_3DE6, 0x8348, 2);
  write_reg(fd, MT9F002_MFR_3DE8, 0xD04C, 2);
  write_reg(fd, MT9F002_MFR_3DEA, 0x8B48, 2);
  write_reg(fd, MT9F002_MFR_3DEC, 0x4641, 2);
  write_reg(fd, MT9F002_MFR_3DEE, 0x4083, 2);
  write_reg(fd, MT9F002_MFR_3DF0, 0x1A00, 2);
  write_reg(fd, MT9F002_MFR_3DF2, 0x8347, 2);
  write_reg(fd, MT9F002_MFR_3DF4, 0x824A, 2);
  write_reg(fd, MT9F002_MFR_3DF6, 0x9A56, 2);
  write_reg(fd, MT9F002_MFR_3DF8, 0x1A00, 2);
  write_reg(fd, MT9F002_MFR_3DFA, 0x951A, 2);
  write_reg(fd, MT9F002_MFR_3DFC, 0x0056, 2);
  write_reg(fd, MT9F002_MFR_3DFE, 0x914D, 2);
  write_reg(fd, MT9F002_MFR_3E00, 0x8B4A, 2);
  write_reg(fd, MT9F002_MFR_3E02, 0x4700, 2);
  write_reg(fd, MT9F002_MFR_3E04, 0x0300, 2);
  write_reg(fd, MT9F002_MFR_3E06, 0x2492, 2);
  write_reg(fd, MT9F002_MFR_3E08, 0x0024, 2);
  write_reg(fd, MT9F002_MFR_3E0A, 0x8A1A, 2);
  write_reg(fd, MT9F002_MFR_3E0C, 0x004F, 2);
  write_reg(fd, MT9F002_MFR_3E0E, 0xB446, 2);
  write_reg(fd, MT9F002_MFR_3E10, 0x8349, 2);
  write_reg(fd, MT9F002_MFR_3E12, 0xB249, 2);
  write_reg(fd, MT9F002_MFR_3E14, 0x4641, 2);
  write_reg(fd, MT9F002_MFR_3E16, 0x408B, 2);
  write_reg(fd, MT9F002_MFR_3E18, 0x4783, 2);
  write_reg(fd, MT9F002_MFR_3E1A, 0x4BDB, 2);
  write_reg(fd, MT9F002_MFR_3E1C, 0x4B47, 2);
  write_reg(fd, MT9F002_MFR_3E1E, 0x4180, 2);
  write_reg(fd, MT9F002_MFR_3E20, 0x502B, 2);
  write_reg(fd, MT9F002_MFR_3E22, 0x4C3A, 2);
  write_reg(fd, MT9F002_MFR_3E24, 0x4180, 2);
  write_reg(fd, MT9F002_MFR_3E26, 0x737C, 2);
  write_reg(fd, MT9F002_MFR_3E28, 0xD124, 2);
  write_reg(fd, MT9F002_MFR_3E2A, 0x9068, 2);
  write_reg(fd, MT9F002_MFR_3E2C, 0x8A20, 2);
  write_reg(fd, MT9F002_MFR_3E2E, 0x2170, 2);
  write_reg(fd, MT9F002_MFR_3E30, 0x8081, 2);
  write_reg(fd, MT9F002_MFR_3E32, 0x6A67, 2);
  write_reg(fd, MT9F002_MFR_3E34, 0x4257, 2);
  write_reg(fd, MT9F002_MFR_3E36, 0x5544, 2);
  write_reg(fd, MT9F002_MFR_3E38, 0x8644, 2);
  write_reg(fd, MT9F002_MFR_3E3A, 0x9755, 2);
  write_reg(fd, MT9F002_MFR_3E3C, 0x5742, 2);
  write_reg(fd, MT9F002_MFR_3E3E, 0x676A, 2);
  write_reg(fd, MT9F002_MFR_3E40, 0x807D, 2);
  write_reg(fd, MT9F002_MFR_3E42, 0x3180, 2);
  write_reg(fd, MT9F002_MFR_3E44, 0x7000, 2);
  write_reg(fd, MT9F002_MFR_3E46, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E48, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E4A, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E4C, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E4E, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E50, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E52, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E54, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E56, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E58, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E5A, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E5C, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E5E, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E60, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E62, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E64, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E66, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E68, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E6A, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E6C, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E6E, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E70, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E72, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E74, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E76, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E78, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E7A, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E7C, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E7E, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E80, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E82, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E84, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E86, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E88, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E8A, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E8C, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E8E, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E90, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E92, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E94, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E96, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E98, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E9A, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E9C, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3E9E, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3EA0, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3EA2, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3EA4, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3EA6, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3EA8, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3EAA, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3EAC, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3EAE, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3EB0, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3EB2, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3EB4, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3EB6, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3EB8, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3EBA, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3EBC, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3EBE, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3EC0, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3EC2, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3EC4, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3EC6, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3EC8, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3ECA, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3176, 0x4000, 2);
  write_reg(fd, MT9F002_MFR_317C, 0xA00A, 2);
  write_reg(fd, MT9F002_MFR_3EE6, 0x0000, 2);
  write_reg(fd, MT9F002_MFR_3ED8, 0xE0E0, 2);
  write_reg(fd, MT9F002_MFR_3EE8, 0x0001, 2);
  write_reg(fd, MT9F002_MFR_3064, 0x0005, 2);
}

/* Configure stage 2 for both MiPi and HiSPi connection */
static void mt9f002_mipi_stage2(int fd)
{
  write_reg(fd, MT9F002_SMIA_TEST, 0x0045, 2);
}

/* Configure stage 1 for parallel connection */
static void mt9f002_parallel_stage1(int fd)
{
  write_reg(fd, MT9F002_RESET_REGISTER , 0x0010, 2);
  write_reg(fd, MT9F002_GLOBAL_GAIN    , 0x1430, 2);
  write_reg(fd, MT9F002_RESET_REGISTER , 0x0010, 2);
  write_reg(fd, MT9F002_RESET_REGISTER , 0x0010, 2);
  write_reg(fd, MT9F002_RESET_REGISTER , 0x0010, 2);
  write_reg(fd, MT9F002_DAC_LD_14_15   , 0xE525, 2);
  write_reg(fd, MT9F002_CTX_CONTROL_REG, 0x0000, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0xF873, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x08AA, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x3219, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x3219, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x3219, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x3200, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x3200, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x3200, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x3200, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x3200, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x1769, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0xA6F3, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0xA6F3, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0xA6F3, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0xA6F3, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0xA6F3, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0xA6F3, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0xA6F3, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0xAFF3, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0xFA64, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0xFA64, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0xFA64, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0xF164, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0xFA64, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0xFA64, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0xFA64, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0xF164, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x276E, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x18CF, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x18CF, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x18CF, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x28CF, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x18CF, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x18CF, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x18CF, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x18CF, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x2363, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x2363, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x2352, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x2363, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x2363, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x2363, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x2352, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x2352, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0xA394, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0xA394, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x8F8F, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0xA3D4, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0xA394, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0xA394, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x8F8F, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x8FCF, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0xDC23, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0xDC63, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0xDC63, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0xDC23, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0xDC23, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0xDC63, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0xDC63, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0xDC23, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x0F73, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x85C0, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x85C0, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x85C0, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x85C0, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x85C0, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x85C0, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x85C0, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x85C4, 2);
  write_reg(fd, MT9F002_CTX_WR_DATA_REG, 0x0000, 2);
  write_reg(fd, MT9F002_ANALOG_CONTROL4, 0x8000, 2);
  write_reg(fd, MT9F002_DAC_LD_14_15   , 0xE525, 2);
  write_reg(fd, MT9F002_DATA_PEDESTAL_ , 0x00A8, 2);
  write_reg(fd, MT9F002_RESET_REGISTER , 0x0090, 2);
  write_reg(fd, MT9F002_SERIAL_FORMAT  , 0x0301, 2);
  write_reg(fd, MT9F002_RESET_REGISTER , 0x1090, 2);
  write_reg(fd, MT9F002_SMIA_TEST      , 0x0845, 2);
  write_reg(fd, MT9F002_RESET_REGISTER , 0x1080, 2);
  write_reg(fd, MT9F002_DATAPATH_SELECT, 0xD880, 2);
  write_reg(fd, MT9F002_RESET_REGISTER , 0x9080, 2);
  write_reg(fd, MT9F002_DATAPATH_SELECT, 0xD880, 2);
  write_reg(fd, MT9F002_RESET_REGISTER , 0x10C8, 2);
  write_reg(fd, MT9F002_DATAPATH_SELECT, 0xD880, 2);
}

/* Configure stage 2 for parallel connection */
static void mt9f002_parallel_stage2(int fd)
{
  write_reg(fd, MT9F002_ANALOG_CONTROL4, 0x8000, 2);
  write_reg(fd, MT9F002_READ_MODE, 0x0041, 2);

  write_reg(fd, READ_MODE              , 0x04C3, 2);
  write_reg(fd, READ_MODE              , 0x04C3, 2);
  write_reg(fd, ANALOG_CONTROL5        , 0x0000, 2);
  write_reg(fd, ANALOG_CONTROL5        , 0x0000, 2);
  write_reg(fd, ANALOG_CONTROL5        , 0x0000, 2);
  write_reg(fd, ANALOG_CONTROL5        , 0x0000, 2);
  write_reg(fd, DAC_LD_28_29           , 0x0047, 2);
  write_reg(fd, COLUMN_CORRECTION      , 0xB080, 2);
  write_reg(fd, COLUMN_CORRECTION      , 0xB100, 2);
  write_reg(fd, DARK_CONTROL3          , 0x0020, 2);
  write_reg(fd, DAC_LD_24_25           , 0x6349, 2);
  write_reg(fd, ANALOG_CONTROL7        , 0x800A, 2);
  write_reg(fd, RESET_REGISTER         , 0x90C8, 2);
  write_reg(fd, CTX_CONTROL_REG        , 0x8005, 2);
  write_reg(fd, ANALOG_CONTROL7        , 0x800A, 2);
  write_reg(fd, DAC_LD_28_29           , 0x0047, 2);
  write_reg(fd, DAC_LD_30_31           , 0x15F0, 2);
  write_reg(fd, DAC_LD_30_31           , 0x15F0, 2);
  write_reg(fd, DAC_LD_30_31           , 0x15F0, 2);
  write_reg(fd, DAC_LD_28_29           , 0x0047, 2);
  write_reg(fd, DAC_LD_28_29           , 0x0047, 2);
  write_reg(fd, RESET_REGISTER         , 0x10C8, 2);
  //write_reg(fd, RESET_REGISTER         , 0x14C8, 2); // reset bad frame
  write_reg(fd, COARSE_INTEGRATION_TIME, 0x08C3, 2);
  write_reg(fd, DIGITAL_TEST           , 0x0000, 2);
  //write_reg(fd, DATAPATH_SELECT        , 0xd881, 2); // permanent line valid
  write_reg(fd, DATAPATH_SELECT        , 0xd880, 2);
  write_reg(fd, READ_MODE              , 0x0041, 2);
  write_reg(fd, X_ODD_INC              , 0x0001, 2);
  write_reg(fd, Y_ODD_INC              , 0x0001, 2);
  write_reg(fd, MASK_CORRUPTED_FRAME   , 0x0001, 2); // 0 output corrupted frame, 1 mask them
}

/* Set the PLL registers based on config */
static void mt9f002_set_pll(int fd)
{
  // Precomputed values to go from InputCLK of (26/2)MHz to 96MHz
  write_reg(fd, MT9F002_VT_PIX_CLK_DIV , pll->vt_pix_clk_div, 2);
  write_reg(fd, MT9F002_VT_SYS_CLK_DIV , pll->vt_sys_clk_div, 2);
  write_reg(fd, MT9F002_PRE_PLL_CLK_DIV, pll->pre_pll_clk_div, 2);
  write_reg(fd, MT9F002_PLL_MULTIPLIER , pll->pll_multiplier, 2);
  write_reg(fd, MT9F002_OP_PIX_CLK_DIV , pll->op_pix_clk_div, 2);
  write_reg(fd, MT9F002_OP_SYS_CLK_DIV , pll->op_sys_clk_div, 2);

  uint16_t smia = read_reg(fd, MT9F002_SMIA_TEST, 2);
  write_reg(fd, MT9F002_SMIA_TEST, (smia & 0xFFBF) | ((pll->shift_vt_pix_clk_div & 0x01)<<6), 2);

  uint16_t row_speed = read_reg(fd, MT9F002_ROW_SPEED, 2);
  row_speed = (row_speed & 0xFFF8) | (pll->rowSpeed_2_0 & 0x07);
  row_speed = (row_speed & 0xF8FF) | ((pll->row_speed_10_8 & 0x07)<<8);
  row_speed = (row_speed&(~0x70)) | (0x2<<4); // from ASIC team : change opclk_delay
  write_reg(fd, MT9F002_ROW_SPEED, row_speed, 2);
}

/**
 * Initialisation of the Aptina MT9F002 CMOS sensor
 * (front camera)
 */
void mt9f002_init(void)
{
  /* Reset the device */
  //TODO???

  /* We open i2c-0 */
  int dev = open("/dev/i2c-0", O_RDWR);
  if (dev < 0) {
    printf("[MT9F002] Could not open i2c-0\r\n");
    return;
  }
  if (ioctl(dev, I2C_SLAVE, MT9V117_ADDRESS) < 0) {
    printf("[MT9F002] Could not change the i2c address to 0x%02X\r\n", MT9V117_ADDRESS);
    close(dev);
    return;
  }

  /* Software reset */
  write_reg(fd, MT9F002_SOFTWARE_RESET, 0x1, 1);
  usleep(1000000); // Wait for one second

  /* Based on the interface configure stage 1 */
  if(interface == HAL_MT9F002_MIPI || interface == HAL_MT9F002_HiSPi) {
    mt9f002_mipi_stage1(dev);
  }
  else {
    mt9f002_parallel_stage1(dev);
  }

  /* Set the PLL based on Input clock and wanted clock frequency */
  mt9f002_set_pll(dev);

  /* Based on the interface configure stage 2 */
  if(interface == HAL_MT9F002_MIPI || interface == HAL_MT9F002_HiSPi) {
    mt9f002_mipi_stage2(dev);
  }
  else {
    mt9f002_parallel_stage2(dev);
  }

  /* Close the device */
  close(dev);
}
