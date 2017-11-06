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
#include "math/pprz_algebra_int.h"
#include "boards/bebop.h"
#include "modules/computer_vision/lib/isp/libisp.h"

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/videodev2.h>
#include <linux/v4l2-mediabus.h>

#define PRINT(string,...) fprintf(stderr, "[MT9F002->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

#if MT9F002_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

#define MAX(x,y) (((x) > (y)) ? (x) : (y))

//#define MT9F002_MAX_WIDTH 4608 //< previous defines
//#define MT9F002_MAX_HEIGHT 3288

/* Camera structure */
struct video_config_t front_camera = {
  .output_size = {
    .w = MT9F002_OUTPUT_WIDTH,
    .h = MT9F002_OUTPUT_HEIGHT
  },
  .sensor_size = {
    .w = MT9F002_OUTPUT_WIDTH,
    .h = MT9F002_OUTPUT_HEIGHT,
  },
  .crop = {
    .x = 0,
    .y = 0,
    .w = MT9F002_OUTPUT_WIDTH,
    .h = MT9F002_OUTPUT_HEIGHT
  },
  .dev_name = "/dev/video1",
  .subdev_name = "/dev/v4l-subdev1",
  .format = V4L2_PIX_FMT_UYVY,
  .subdev_format = V4L2_MBUS_FMT_SGRBG10_1X10,
  .buf_cnt = 5,
  .filters = VIDEO_FILTER_ISP,
  .cv_listener = NULL,
  .fps = MT9F002_TARGET_FPS
};

#define max(m1,m2) \
  ({ __typeof__ (m1) _m1 = (m1); \
    __typeof__ (m2) _m2 = (m2); \
    _m1 > _m2 ? _m1 : _m2; })

#define min(m1,m2) \
  ({ __typeof__ (m1) _m1 = (m1); \
    __typeof__ (m2) _m2 = (m2); \
    _m1 < _m2 ? _m1 : _m2; })

#define clamp(c1,l,u) \
  ({ __typeof__ (c1) _c1 = (c1); \
    __typeof__ (l) _l = (l); \
    __typeof__ (u) _u = (u); \
    _c1 < _l ? _l : (_c1 > _u ? _u : _c1); })

#define clamp_t(t, c1, l, u) \
  ({ __typeof__ (t) _c1 = (c1); \
    __typeof__ (t) _l = (l); \
    __typeof__ (t) _u = (u); \
    _c1 < _l ? _l : (_c1 > _u ? _u : _c1); })

#define ALIGN(x,a1) \
  ({ __typeof__ (x) _x = (x); \
    __typeof__ (a1) _a1 = (a1); \
    __typeof__ (a1) _r = _x%_a1; \
    _r ? _x + (_a1 - _r) : _x; })

struct blanking_ {
  uint16_t min_line_blanking_pck;
  uint16_t min_line_length_pck;
  uint16_t min_line_fifo_pck;
  uint16_t fine_integration_time_min;
  uint16_t fine_integration_time_max_margin;
} mt9f002_blanking;

/**
 * Write multiple bytes to a single register
 */
static void write_reg(struct mt9f002_t *mt, uint16_t addr, uint32_t val, uint8_t len)
{
  mt->i2c_trans.buf[0] = addr >> 8;
  mt->i2c_trans.buf[1] = addr & 0xFF;

  // Fix sigdness based on length
  if (len == 1) {
    mt->i2c_trans.buf[2] = val & 0xFF;
  } else if (len == 2) {
    mt->i2c_trans.buf[2] = (val >> 8) & 0xFF;
    mt->i2c_trans.buf[3] = val & 0xFF;
  } else if (len == 4) {
    mt->i2c_trans.buf[2] = (val >> 24) & 0xFF;
    mt->i2c_trans.buf[3] = (val >> 16) & 0xFF;
    mt->i2c_trans.buf[4] = (val >> 8) & 0xFF;
    mt->i2c_trans.buf[5] = val & 0xFF;
  } else {
    printf("[MT9F002] write_reg with incorrect length %d\r\n", len);
  }

  // Transmit the buffer
  i2c_transmit(mt->i2c_periph, &mt->i2c_trans, MT9F002_ADDRESS, len + 2);
}

/**
 * Read multiple bytes from a register
 */
static uint32_t read_reg(struct mt9f002_t *mt, uint16_t addr, uint8_t len)
{
  uint32_t ret = 0;
  mt->i2c_trans.buf[0] = addr >> 8;
  mt->i2c_trans.buf[1] = addr & 0xFF;

  // Transmit the buffer and receive back
  i2c_transceive(mt->i2c_periph, &mt->i2c_trans, MT9F002_ADDRESS, 2, len);

  /* Fix sigdness */
  for (uint8_t i = 0; i < len; i++) {
    ret |= mt->i2c_trans.buf[len - i - 1] << (8 * i);
  }
  return ret;
}

/**
 * Configure stage 1 for both MiPi and HiSPi connection
 */
static inline void mt9f002_mipi_stage1(struct mt9f002_t *mt)
{
  write_reg(mt, MT9F002_RESET_REGISTER, 0x0118, 2);
  write_reg(mt, MT9F002_MODE_SELECT, 0x00, 1);

  uint32_t serialFormat;
  if (mt->interface == MT9F002_HiSPi) {
    serialFormat = (3 << 8) | 2; // 2 Serial lanes
  } else {
    serialFormat = (2 << 8) | 2; // 2 Serial lanes
  }
  write_reg(mt, MT9F002_SERIAL_FORMAT, serialFormat, 2);
  uint32_t dataFormat = (8 << 8) | 8; // 8 Bits pixel depth
  write_reg(mt, MT9F002_CPP_DATA_FORMAT, dataFormat, 2);

  write_reg(mt, MT9F002_MFR_3D00, 0x0435, 2);
  write_reg(mt, MT9F002_MFR_3D02, 0x435D, 2);
  write_reg(mt, MT9F002_MFR_3D04, 0x6698, 2);
  write_reg(mt, MT9F002_MFR_3D06, 0xFFFF, 2);
  write_reg(mt, MT9F002_MFR_3D08, 0x7783, 2);
  write_reg(mt, MT9F002_MFR_3D0A, 0x101B, 2);
  write_reg(mt, MT9F002_MFR_3D0C, 0x732C, 2);
  write_reg(mt, MT9F002_MFR_3D0E, 0x4230, 2);
  write_reg(mt, MT9F002_MFR_3D10, 0x5881, 2);
  write_reg(mt, MT9F002_MFR_3D12, 0x5C3A, 2);
  write_reg(mt, MT9F002_MFR_3D14, 0x0140, 2);
  write_reg(mt, MT9F002_MFR_3D16, 0x2300, 2);
  write_reg(mt, MT9F002_MFR_3D18, 0x815F, 2);
  write_reg(mt, MT9F002_MFR_3D1A, 0x6789, 2);
  write_reg(mt, MT9F002_MFR_3D1C, 0x5920, 2);
  write_reg(mt, MT9F002_MFR_3D1E, 0x0C20, 2);
  write_reg(mt, MT9F002_MFR_3D20, 0x21C0, 2);
  write_reg(mt, MT9F002_MFR_3D22, 0x4684, 2);
  write_reg(mt, MT9F002_MFR_3D24, 0x4892, 2);
  write_reg(mt, MT9F002_MFR_3D26, 0x1A00, 2);
  write_reg(mt, MT9F002_MFR_3D28, 0xBA4C, 2);
  write_reg(mt, MT9F002_MFR_3D2A, 0x8D48, 2);
  write_reg(mt, MT9F002_MFR_3D2C, 0x4641, 2);
  write_reg(mt, MT9F002_MFR_3D2E, 0x408C, 2);
  write_reg(mt, MT9F002_MFR_3D30, 0x4784, 2);
  write_reg(mt, MT9F002_MFR_3D32, 0x4A87, 2);
  write_reg(mt, MT9F002_MFR_3D34, 0x561A, 2);
  write_reg(mt, MT9F002_MFR_3D36, 0x00A5, 2);
  write_reg(mt, MT9F002_MFR_3D38, 0x1A00, 2);
  write_reg(mt, MT9F002_MFR_3D3A, 0x5693, 2);
  write_reg(mt, MT9F002_MFR_3D3C, 0x4D8D, 2);
  write_reg(mt, MT9F002_MFR_3D3E, 0x4A47, 2);
  write_reg(mt, MT9F002_MFR_3D40, 0x4041, 2);
  write_reg(mt, MT9F002_MFR_3D42, 0x8200, 2);
  write_reg(mt, MT9F002_MFR_3D44, 0x24B7, 2);
  write_reg(mt, MT9F002_MFR_3D46, 0x0024, 2);
  write_reg(mt, MT9F002_MFR_3D48, 0x8D4F, 2);
  write_reg(mt, MT9F002_MFR_3D4A, 0x831A, 2);
  write_reg(mt, MT9F002_MFR_3D4C, 0x00B4, 2);
  write_reg(mt, MT9F002_MFR_3D4E, 0x4684, 2);
  write_reg(mt, MT9F002_MFR_3D50, 0x49CE, 2);
  write_reg(mt, MT9F002_MFR_3D52, 0x4946, 2);
  write_reg(mt, MT9F002_MFR_3D54, 0x4140, 2);
  write_reg(mt, MT9F002_MFR_3D56, 0x9247, 2);
  write_reg(mt, MT9F002_MFR_3D58, 0x844B, 2);
  write_reg(mt, MT9F002_MFR_3D5A, 0xCE4B, 2);
  write_reg(mt, MT9F002_MFR_3D5C, 0x4741, 2);
  write_reg(mt, MT9F002_MFR_3D5E, 0x502F, 2);
  write_reg(mt, MT9F002_MFR_3D60, 0xBD3A, 2);
  write_reg(mt, MT9F002_MFR_3D62, 0x5181, 2);
  write_reg(mt, MT9F002_MFR_3D64, 0x5E73, 2);
  write_reg(mt, MT9F002_MFR_3D66, 0x7C0A, 2);
  write_reg(mt, MT9F002_MFR_3D68, 0x7770, 2);
  write_reg(mt, MT9F002_MFR_3D6A, 0x8085, 2);
  write_reg(mt, MT9F002_MFR_3D6C, 0x6A82, 2);
  write_reg(mt, MT9F002_MFR_3D6E, 0x6742, 2);
  write_reg(mt, MT9F002_MFR_3D70, 0x8244, 2);
  write_reg(mt, MT9F002_MFR_3D72, 0x831A, 2);
  write_reg(mt, MT9F002_MFR_3D74, 0x0099, 2);
  write_reg(mt, MT9F002_MFR_3D76, 0x44DF, 2);
  write_reg(mt, MT9F002_MFR_3D78, 0x1A00, 2);
  write_reg(mt, MT9F002_MFR_3D7A, 0x8542, 2);
  write_reg(mt, MT9F002_MFR_3D7C, 0x8567, 2);
  write_reg(mt, MT9F002_MFR_3D7E, 0x826A, 2);
  write_reg(mt, MT9F002_MFR_3D80, 0x857C, 2);
  write_reg(mt, MT9F002_MFR_3D82, 0x6B80, 2);
  write_reg(mt, MT9F002_MFR_3D84, 0x7000, 2);
  write_reg(mt, MT9F002_MFR_3D86, 0xB831, 2);
  write_reg(mt, MT9F002_MFR_3D88, 0x40BE, 2);
  write_reg(mt, MT9F002_MFR_3D8A, 0x6700, 2);
  write_reg(mt, MT9F002_MFR_3D8C, 0x0CBD, 2);
  write_reg(mt, MT9F002_MFR_3D8E, 0x4482, 2);
  write_reg(mt, MT9F002_MFR_3D90, 0x7898, 2);
  write_reg(mt, MT9F002_MFR_3D92, 0x7480, 2);
  write_reg(mt, MT9F002_MFR_3D94, 0x5680, 2);
  write_reg(mt, MT9F002_MFR_3D96, 0x9755, 2);
  write_reg(mt, MT9F002_MFR_3D98, 0x8057, 2);
  write_reg(mt, MT9F002_MFR_3D9A, 0x8056, 2);
  write_reg(mt, MT9F002_MFR_3D9C, 0x9256, 2);
  write_reg(mt, MT9F002_MFR_3D9E, 0x8057, 2);
  write_reg(mt, MT9F002_MFR_3DA0, 0x8055, 2);
  write_reg(mt, MT9F002_MFR_3DA2, 0x817C, 2);
  write_reg(mt, MT9F002_MFR_3DA4, 0x969B, 2);
  write_reg(mt, MT9F002_MFR_3DA6, 0x56A6, 2);
  write_reg(mt, MT9F002_MFR_3DA8, 0x44BE, 2);
  write_reg(mt, MT9F002_MFR_3DAA, 0x000C, 2);
  write_reg(mt, MT9F002_MFR_3DAC, 0x867A, 2);
  write_reg(mt, MT9F002_MFR_3DAE, 0x9474, 2);
  write_reg(mt, MT9F002_MFR_3DB0, 0x8A79, 2);
  write_reg(mt, MT9F002_MFR_3DB2, 0x9367, 2);
  write_reg(mt, MT9F002_MFR_3DB4, 0xBF6A, 2);
  write_reg(mt, MT9F002_MFR_3DB6, 0x816C, 2);
  write_reg(mt, MT9F002_MFR_3DB8, 0x8570, 2);
  write_reg(mt, MT9F002_MFR_3DBA, 0x836C, 2);
  write_reg(mt, MT9F002_MFR_3DBC, 0x826A, 2);
  write_reg(mt, MT9F002_MFR_3DBE, 0x8245, 2);
  write_reg(mt, MT9F002_MFR_3DC0, 0xFFFF, 2);
  write_reg(mt, MT9F002_MFR_3DC2, 0xFFD6, 2);
  write_reg(mt, MT9F002_MFR_3DC4, 0x4582, 2);
  write_reg(mt, MT9F002_MFR_3DC6, 0x6A82, 2);
  write_reg(mt, MT9F002_MFR_3DC8, 0x6C83, 2);
  write_reg(mt, MT9F002_MFR_3DCA, 0x7000, 2);
  write_reg(mt, MT9F002_MFR_3DCC, 0x8024, 2);
  write_reg(mt, MT9F002_MFR_3DCE, 0xB181, 2);
  write_reg(mt, MT9F002_MFR_3DD0, 0x6859, 2);
  write_reg(mt, MT9F002_MFR_3DD2, 0x732B, 2);
  write_reg(mt, MT9F002_MFR_3DD4, 0x4030, 2);
  write_reg(mt, MT9F002_MFR_3DD6, 0x4982, 2);
  write_reg(mt, MT9F002_MFR_3DD8, 0x101B, 2);
  write_reg(mt, MT9F002_MFR_3DDA, 0x4083, 2);
  write_reg(mt, MT9F002_MFR_3DDC, 0x6785, 2);
  write_reg(mt, MT9F002_MFR_3DDE, 0x3A00, 2);
  write_reg(mt, MT9F002_MFR_3DE0, 0x8820, 2);
  write_reg(mt, MT9F002_MFR_3DE2, 0x0C59, 2);
  write_reg(mt, MT9F002_MFR_3DE4, 0x8546, 2);
  write_reg(mt, MT9F002_MFR_3DE6, 0x8348, 2);
  write_reg(mt, MT9F002_MFR_3DE8, 0xD04C, 2);
  write_reg(mt, MT9F002_MFR_3DEA, 0x8B48, 2);
  write_reg(mt, MT9F002_MFR_3DEC, 0x4641, 2);
  write_reg(mt, MT9F002_MFR_3DEE, 0x4083, 2);
  write_reg(mt, MT9F002_MFR_3DF0, 0x1A00, 2);
  write_reg(mt, MT9F002_MFR_3DF2, 0x8347, 2);
  write_reg(mt, MT9F002_MFR_3DF4, 0x824A, 2);
  write_reg(mt, MT9F002_MFR_3DF6, 0x9A56, 2);
  write_reg(mt, MT9F002_MFR_3DF8, 0x1A00, 2);
  write_reg(mt, MT9F002_MFR_3DFA, 0x951A, 2);
  write_reg(mt, MT9F002_MFR_3DFC, 0x0056, 2);
  write_reg(mt, MT9F002_MFR_3DFE, 0x914D, 2);
  write_reg(mt, MT9F002_MFR_3E00, 0x8B4A, 2);
  write_reg(mt, MT9F002_MFR_3E02, 0x4700, 2);
  write_reg(mt, MT9F002_MFR_3E04, 0x0300, 2);
  write_reg(mt, MT9F002_MFR_3E06, 0x2492, 2);
  write_reg(mt, MT9F002_MFR_3E08, 0x0024, 2);
  write_reg(mt, MT9F002_MFR_3E0A, 0x8A1A, 2);
  write_reg(mt, MT9F002_MFR_3E0C, 0x004F, 2);
  write_reg(mt, MT9F002_MFR_3E0E, 0xB446, 2);
  write_reg(mt, MT9F002_MFR_3E10, 0x8349, 2);
  write_reg(mt, MT9F002_MFR_3E12, 0xB249, 2);
  write_reg(mt, MT9F002_MFR_3E14, 0x4641, 2);
  write_reg(mt, MT9F002_MFR_3E16, 0x408B, 2);
  write_reg(mt, MT9F002_MFR_3E18, 0x4783, 2);
  write_reg(mt, MT9F002_MFR_3E1A, 0x4BDB, 2);
  write_reg(mt, MT9F002_MFR_3E1C, 0x4B47, 2);
  write_reg(mt, MT9F002_MFR_3E1E, 0x4180, 2);
  write_reg(mt, MT9F002_MFR_3E20, 0x502B, 2);
  write_reg(mt, MT9F002_MFR_3E22, 0x4C3A, 2);
  write_reg(mt, MT9F002_MFR_3E24, 0x4180, 2);
  write_reg(mt, MT9F002_MFR_3E26, 0x737C, 2);
  write_reg(mt, MT9F002_MFR_3E28, 0xD124, 2);
  write_reg(mt, MT9F002_MFR_3E2A, 0x9068, 2);
  write_reg(mt, MT9F002_MFR_3E2C, 0x8A20, 2);
  write_reg(mt, MT9F002_MFR_3E2E, 0x2170, 2);
  write_reg(mt, MT9F002_MFR_3E30, 0x8081, 2);
  write_reg(mt, MT9F002_MFR_3E32, 0x6A67, 2);
  write_reg(mt, MT9F002_MFR_3E34, 0x4257, 2);
  write_reg(mt, MT9F002_MFR_3E36, 0x5544, 2);
  write_reg(mt, MT9F002_MFR_3E38, 0x8644, 2);
  write_reg(mt, MT9F002_MFR_3E3A, 0x9755, 2);
  write_reg(mt, MT9F002_MFR_3E3C, 0x5742, 2);
  write_reg(mt, MT9F002_MFR_3E3E, 0x676A, 2);
  write_reg(mt, MT9F002_MFR_3E40, 0x807D, 2);
  write_reg(mt, MT9F002_MFR_3E42, 0x3180, 2);
  write_reg(mt, MT9F002_MFR_3E44, 0x7000, 2);
  write_reg(mt, MT9F002_MFR_3E46, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E48, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E4A, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E4C, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E4E, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E50, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E52, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E54, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E56, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E58, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E5A, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E5C, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E5E, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E60, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E62, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E64, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E66, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E68, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E6A, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E6C, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E6E, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E70, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E72, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E74, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E76, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E78, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E7A, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E7C, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E7E, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E80, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E82, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E84, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E86, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E88, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E8A, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E8C, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E8E, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E90, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E92, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E94, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E96, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E98, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E9A, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E9C, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3E9E, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3EA0, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3EA2, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3EA4, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3EA6, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3EA8, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3EAA, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3EAC, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3EAE, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3EB0, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3EB2, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3EB4, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3EB6, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3EB8, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3EBA, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3EBC, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3EBE, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3EC0, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3EC2, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3EC4, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3EC6, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3EC8, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3ECA, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3176, 0x4000, 2);
  write_reg(mt, MT9F002_MFR_317C, 0xA00A, 2);
  write_reg(mt, MT9F002_MFR_3EE6, 0x0000, 2);
  write_reg(mt, MT9F002_MFR_3ED8, 0xE0E0, 2);
  write_reg(mt, MT9F002_MFR_3EE8, 0x0001, 2);
  write_reg(mt, MT9F002_SMIA_TEST, 0x0005, 2);
}

/**
 * Configure stage 2 for both MiPi and HiSPi connection
 */
static inline void mt9f002_mipi_stage2(struct mt9f002_t *mt)
{
  write_reg(mt, MT9F002_SMIA_TEST, 0x0045, 2);
}

/**
 * Configure stage 3 for both MiPi and HiSPi connection
 */
static inline void mt9f002_mipi_stage3(struct mt9f002_t *mt)
{
  write_reg(mt, MT9F002_EXTRA_DELAY      , 0x0000, 2);
  write_reg(mt, MT9F002_RESET_REGISTER   , 0x0118, 2);
  write_reg(mt, MT9F002_MFR_3EDC, 0x68CF, 2);
  write_reg(mt, MT9F002_MFR_3EE2, 0xE363, 2);
}

/**
 * Configure stage 1 for parallel connection
 */
static inline void mt9f002_parallel_stage1(struct mt9f002_t *mt)
{
  write_reg(mt, MT9F002_RESET_REGISTER , 0x0010, 2);
  write_reg(mt, MT9F002_GLOBAL_GAIN    , 0x1430, 2);
  write_reg(mt, MT9F002_RESET_REGISTER , 0x0010, 2);
  write_reg(mt, MT9F002_RESET_REGISTER , 0x0010, 2);
  write_reg(mt, MT9F002_RESET_REGISTER , 0x0010, 2);
  write_reg(mt, MT9F002_DAC_LD_14_15   , 0xE525, 2);
  write_reg(mt, MT9F002_CTX_CONTROL_REG, 0x0000, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0xF873, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x08AA, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x3219, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x3219, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x3219, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x3200, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x3200, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x3200, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x3200, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x3200, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x1769, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0xA6F3, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0xA6F3, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0xA6F3, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0xA6F3, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0xA6F3, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0xA6F3, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0xA6F3, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0xAFF3, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0xFA64, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0xFA64, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0xFA64, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0xF164, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0xFA64, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0xFA64, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0xFA64, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0xF164, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x276E, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x18CF, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x18CF, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x18CF, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x28CF, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x18CF, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x18CF, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x18CF, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x18CF, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x2363, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x2363, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x2352, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x2363, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x2363, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x2363, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x2352, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x2352, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0xA394, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0xA394, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x8F8F, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0xA3D4, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0xA394, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0xA394, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x8F8F, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x8FCF, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0xDC23, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0xDC63, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0xDC63, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0xDC23, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0xDC23, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0xDC63, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0xDC63, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0xDC23, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x0F73, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x85C0, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x85C0, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x85C0, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x85C0, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x85C0, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x85C0, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x85C0, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x85C4, 2);
  write_reg(mt, MT9F002_CTX_WR_DATA_REG, 0x0000, 2);
  write_reg(mt, MT9F002_ANALOG_CONTROL4, 0x8000, 2);
  write_reg(mt, MT9F002_DAC_LD_14_15   , 0xE525, 2);
  write_reg(mt, MT9F002_DATA_PEDESTAL_ , 0x00A8, 2);
  write_reg(mt, MT9F002_RESET_REGISTER , 0x0090, 2);
  write_reg(mt, MT9F002_SERIAL_FORMAT  , 0x0301, 2);
  write_reg(mt, MT9F002_RESET_REGISTER , 0x1090, 2);
  write_reg(mt, MT9F002_SMIA_TEST      , 0x0845, 2);
  write_reg(mt, MT9F002_RESET_REGISTER , 0x1080, 2);
  write_reg(mt, MT9F002_DATAPATH_SELECT, 0xD880, 2);
  write_reg(mt, MT9F002_RESET_REGISTER , 0x9080, 2);
  write_reg(mt, MT9F002_DATAPATH_SELECT, 0xD880, 2);
  write_reg(mt, MT9F002_RESET_REGISTER , 0x10C8, 2);
  write_reg(mt, MT9F002_DATAPATH_SELECT, 0xD880, 2);
}

/**
 * Configure stage 2 for parallel connection
 */
static inline void mt9f002_parallel_stage2(struct mt9f002_t *mt)
{
  write_reg(mt, MT9F002_ANALOG_CONTROL4, 0x8000, 2);
  write_reg(mt, MT9F002_READ_MODE, 0x0041, 2);

  write_reg(mt, MT9F002_READ_MODE              , 0x04C3, 2);
  write_reg(mt, MT9F002_READ_MODE              , 0x04C3, 2);
  write_reg(mt, MT9F002_ANALOG_CONTROL5        , 0x0000, 2);
  write_reg(mt, MT9F002_ANALOG_CONTROL5        , 0x0000, 2);
  write_reg(mt, MT9F002_ANALOG_CONTROL5        , 0x0000, 2);
  write_reg(mt, MT9F002_ANALOG_CONTROL5        , 0x0000, 2);
  write_reg(mt, MT9F002_DAC_LD_28_29           , 0x0047, 2);
  write_reg(mt, MT9F002_COLUMN_CORRECTION      , 0xB080, 2);
  write_reg(mt, MT9F002_COLUMN_CORRECTION      , 0xB100, 2);
  write_reg(mt, MT9F002_DARK_CONTROL3          , 0x0020, 2);
  write_reg(mt, MT9F002_DAC_LD_24_25           , 0x6349, 2);
  write_reg(mt, MT9F002_ANALOG_CONTROL7        , 0x800A, 2);
  write_reg(mt, MT9F002_RESET_REGISTER         , 0x90C8, 2);
  write_reg(mt, MT9F002_CTX_CONTROL_REG        , 0x8005, 2);
  write_reg(mt, MT9F002_ANALOG_CONTROL7        , 0x800A, 2);
  write_reg(mt, MT9F002_DAC_LD_28_29           , 0x0047, 2);
  write_reg(mt, MT9F002_DAC_LD_30_31           , 0x15F0, 2);
  write_reg(mt, MT9F002_DAC_LD_30_31           , 0x15F0, 2);
  write_reg(mt, MT9F002_DAC_LD_30_31           , 0x15F0, 2);
  write_reg(mt, MT9F002_DAC_LD_28_29           , 0x0047, 2);
  write_reg(mt, MT9F002_DAC_LD_28_29           , 0x0047, 2);
  write_reg(mt, MT9F002_RESET_REGISTER         , 0x10C8, 2);
  //write_reg(mt, MT9F002_RESET_REGISTER         , 0x14C8, 2); // reset bad frame
  write_reg(mt, MT9F002_COARSE_INTEGRATION_TIME, 0x08C3, 2);
  write_reg(mt, MT9F002_DIGITAL_TEST           , 0x0000, 2);
  //write_reg(mt, MT9F002_DATAPATH_SELECT        , 0xd881, 2); // permanent line valid
  write_reg(mt, MT9F002_DATAPATH_SELECT        , 0xd880, 2);

  if (mt->x_odd_inc > 1 && mt->y_odd_inc > 1) {
    write_reg(mt, MT9F002_READ_MODE     , 0x0441, 2);
    write_reg(mt, MT9F002_X_ODD_INC     , mt->x_odd_inc, 2);
    write_reg(mt, MT9F002_Y_ODD_INC     , mt->y_odd_inc, 2);
    // bayer resampling
    write_reg(mt, MT9F002_SCALING_MODE    , 2, 2);
    write_reg(mt, MT9F002_DATAPATH_SELECT   , 0xd8b0, 2);
  } else {
    write_reg(mt, MT9F002_READ_MODE              , 0x0041, 2);
    write_reg(mt, MT9F002_X_ODD_INC              , mt->x_odd_inc, 2);
    write_reg(mt, MT9F002_Y_ODD_INC              , mt->y_odd_inc, 2);
  }
  write_reg(mt, MT9F002_MASK_CORRUPTED_FRAMES  , 0x0001, 1); // 0 output corrupted frame, 1 mask them
}

/**
 * Set the PLL registers based on config
 */
static inline void mt9f002_set_pll(struct mt9f002_t *mt)
{
  // Update registers
  write_reg(mt, MT9F002_VT_PIX_CLK_DIV , mt->vt_pix_clk_div, 2);
  write_reg(mt, MT9F002_VT_SYS_CLK_DIV , mt->vt_sys_clk_div, 2);
  write_reg(mt, MT9F002_PRE_PLL_CLK_DIV, mt->pre_pll_clk_div, 2);
  write_reg(mt, MT9F002_PLL_MULTIPLIER , mt->pll_multiplier, 2);
  write_reg(mt, MT9F002_OP_PIX_CLK_DIV , mt->op_pix_clk_div, 2);
  write_reg(mt, MT9F002_OP_SYS_CLK_DIV , mt->op_sys_clk_div, 2);

  uint16_t smia = read_reg(mt, MT9F002_SMIA_TEST, 2);
  write_reg(mt, MT9F002_SMIA_TEST, (smia & 0xFFBF) | (mt->shift_vt_pix_clk_div << 6), 2); // shift_vt_pix_clk_div

  uint16_t row_speed = read_reg(mt, MT9F002_ROW_SPEED, 2);
  row_speed = (row_speed & 0xFFF8) | (mt->rowSpeed_2_0 & 0x07); // rowSpeed_2_0
  row_speed = (row_speed & 0xF8FF) | ((mt->row_speed_10_8 & 0x07) << 8); // row_speed_10_8
  row_speed = (row_speed & (~0x70)) | (0x2 << 4); // Change opclk_delay
  write_reg(mt, MT9F002_ROW_SPEED, row_speed, 2);

  // Compute clocks
  mt->vt_pix_clk = mt->input_clk_freq * (float)mt->pll_multiplier * (float)(1 + mt->shift_vt_pix_clk_div)
                   / ((float)mt->pre_pll_clk_div * (float)mt->vt_sys_clk_div * (float)mt->vt_pix_clk_div);
  mt->op_pix_clk = mt->input_clk_freq * (float)mt->pll_multiplier
                   / ((float)mt->pre_pll_clk_div * (float)mt->op_sys_clk_div * (float)mt->op_pix_clk_div);
}

/**
 *Set the blanking configuration
 * Blanking of the MT9F002 depends on the target FPS
 */
static void mt9f002_blanking_init(struct mt9f002_t *mt)
{
  /*
  #define NO_BINNING_NO_SKIPPING            0x1
  #define BINNING_ONLY                0x3
  #define BINNING_AND_SKIPPING            0x7 */
  if (mt->x_odd_inc > 1) {
    if (mt->y_odd_inc > 1) {
      /* Binning XY */
      mt9f002_blanking.min_line_blanking_pck        = 2950;
      mt9f002_blanking.min_line_length_pck        = 4650;
      mt9f002_blanking.min_line_fifo_pck          = 120;
      mt9f002_blanking.fine_integration_time_max_margin   = 2000;
      mt9f002_blanking.fine_integration_time_min      = 2200;
    } else {
      /* Binning X */
      mt9f002_blanking.min_line_blanking_pck        = 0;
      mt9f002_blanking.min_line_length_pck        = 3495;
      mt9f002_blanking.min_line_fifo_pck          = 60;
      mt9f002_blanking.fine_integration_time_max_margin   = 1500;
      mt9f002_blanking.fine_integration_time_min      = 1900;
    }
  } else {
    if (mt->output_scaler != 1) {
      /* Scaler mode */
      mt9f002_blanking.min_line_blanking_pck        = 2400;
      mt9f002_blanking.min_line_length_pck        = 1750;
      mt9f002_blanking.min_line_fifo_pck          = 60;
      mt9f002_blanking.fine_integration_time_max_margin   = 1316;
      mt9f002_blanking.fine_integration_time_min      = 1032;
    } else {
      /* Normal mode */
      mt9f002_blanking.min_line_blanking_pck        = 1316;
      mt9f002_blanking.min_line_length_pck        = 1032;
      mt9f002_blanking.min_line_fifo_pck          = 60;
      mt9f002_blanking.fine_integration_time_max_margin   = 1316;
      mt9f002_blanking.fine_integration_time_min      = 1032;
    }
  }

  return;
}

static void mt9f002_set_blanking(struct mt9f002_t *mt)
{
  /* Read some config values in order to calculate blanking configuration */
  uint16_t min_line_blanking_pck  = mt9f002_blanking.min_line_blanking_pck;
  uint16_t min_line_length_pck      = mt9f002_blanking.min_line_length_pck;
  uint16_t min_line_fifo_pck    = mt9f002_blanking.min_line_fifo_pck;

  //uint16_t min_line_fifo_pck        = 0x005E;
  //uint16_t min_line_blanking_pck    = read_reg(mt, MT9F002_MIN_LINE_BLANKING_PCK, 2);
  //uint16_t min_line_length_pck      = read_reg(mt, MT9F002_MIN_LINE_LENGTH_PCK, 2);
  uint16_t min_frame_blanking_lines = read_reg(mt, MT9F002_MIN_FRAME_BLANKING_LINES, 2);
  float subsamplingX_factor          = (float)(1 + mt->x_odd_inc) / 2.0f; // See page 52
  float subsamplingY_factor          = (float)(1 + mt->y_odd_inc) / 2.0f; // See page 52

  /* Calculate minimum line length */
  uint16_t min_line_length = MAX(min_line_length_pck,
                                 ((uint16_t)((mt->sensor_width - 1 + mt->x_odd_inc) / subsamplingX_factor)) + min_line_blanking_pck); // EQ 9
  if (mt->interface == MT9F002_MIPI ||
      mt->interface == MT9F002_HiSPi) {
    min_line_length = MAX(min_line_length,
                          ((uint16_t)((float)mt->output_width * mt->vt_pix_clk / mt->op_pix_clk) / 2) + min_line_fifo_pck); // 2 lanes, pll clocks
  } else {
    min_line_length = MAX(min_line_length,
                          ((uint16_t)((float)mt->output_width * mt->vt_pix_clk / mt->op_pix_clk)) + min_line_fifo_pck); // pll clocks
  }

  /* Do some magic to get it to work with P7 ISP (with horizontal blanking) */
  uint32_t clkRatio_num = mt->op_sys_clk_div * mt->op_pix_clk_div * mt->row_speed_10_8 * (1 + mt->shift_vt_pix_clk_div);
  uint32_t clkRatio_den = mt->vt_sys_clk_div * mt->vt_pix_clk_div;

  /* Divide by the GCD to find smallest ratio */
  uint32_t clkRatio_gcd = int32_gcd(clkRatio_num, clkRatio_den);
  clkRatio_num = clkRatio_num / clkRatio_gcd;
  clkRatio_den = clkRatio_den / clkRatio_gcd;

  /* Calculate minimum horizontal blanking, since fpga line_length must be divideable by 2 */
  uint32_t min_horizontal_blanking = clkRatio_num;
  if ((clkRatio_den % 2) != 0) {
    min_horizontal_blanking *= 2;
  }
  /* Fix fpga correction based on min horizontal blanking */
  if ((min_line_length % min_horizontal_blanking) != 0) {
    min_line_length += min_horizontal_blanking - (min_line_length % min_horizontal_blanking);
  }

  /* Calculate minimum frame length lines */
  uint16_t min_frame_length = (mt->sensor_height) / subsamplingY_factor + min_frame_blanking_lines; // (EQ 10)
  /* Calculate FPS we get using these minimums (Maximum FPS) */
  mt->line_length = min_line_length;
  mt->frame_length = min_frame_length;
  mt->real_fps = mt->vt_pix_clk * 1000000 / (float)(mt->line_length * mt->frame_length);
  VERBOSE_PRINT("Maximum FPS: %0.3f\n", mt->real_fps);
  /* Check if we need to downscale the FPS and bruteforce better solution */
  if (mt->target_fps < mt->real_fps) {
    float min_fps_err = fabs(mt->target_fps - mt->real_fps);
    float new_fps = mt->real_fps;

    // Go through all possible line lengths
    for (uint16_t ll = min_line_length; ll <= MT9F002_LINE_LENGTH_MAX; ll += min_horizontal_blanking) {
      // Go through all possible frame lengths
      for (uint16_t fl = min_frame_length; fl < MT9F002_FRAME_LENGTH_MAX; ++fl) {
        new_fps = mt->vt_pix_clk * 1000000 / (float)(ll * fl);

        // Calculate FPS error and save if it is better
        float fps_err = fabs(mt->target_fps - new_fps);
        if (fps_err < min_fps_err) {
          min_fps_err = fps_err;
          mt->line_length   = ll;
          mt->frame_length  = fl;
          mt->real_fps      = new_fps;
        }

        // Stop searching if FPS is lower or equal
        if (mt->target_fps >= new_fps) {
          break;
        }
      }

      // Calculate if next step is still needed (since we only need to go one step below target_fps)
      new_fps = mt->vt_pix_clk * 1000000 / (float)(ll * min_frame_length);

      // Stop searching if FPS is lower or equal
      if (mt->target_fps > new_fps) {
        break;
      }
    }
  }

  /* Actually set the calculated values */
  write_reg(mt, MT9F002_LINE_LENGTH_PCK, mt->line_length, 2);
  write_reg(mt, MT9F002_FRAME_LENGTH_LINES, mt->frame_length, 2);
}

/**
 * Set the exposure configuration
 * Depends on the blanking (and therefore the FPS)
 */
void mt9f002_set_exposure(struct mt9f002_t *mt)
{
  /* Fetch minimum and maximum integration times */
  uint16_t coarse_integration_min = read_reg(mt, MT9F002_COARSE_INTEGRATION_TIME_MIN, 2);
  uint16_t coarse_integration_max = mt->frame_length - read_reg(mt, MT9F002_COARSE_INTEGRATION_TIME_MAX_MARGIN, 2);
  uint16_t fine_integration_min = mt9f002_blanking.fine_integration_time_min;
  uint16_t fine_integration_max_margin = mt9f002_blanking.fine_integration_time_max_margin;
  uint16_t fine_integration_max = mt->line_length - fine_integration_max_margin;

  /* Compute fine and coarse integration time */
  uint32_t integration = mt->target_exposure * mt->vt_pix_clk * 1000;
  uint16_t coarse_integration = integration / mt->line_length;
  uint16_t fine_integration = integration % mt->line_length;

  /* Make sure fine integration is inside bounds */
  if (fine_integration_min > fine_integration || fine_integration > fine_integration_max) {
    int32_t upper_coarse_integration = coarse_integration + 1;
    int32_t upper_fine_integration = fine_integration_min;

    int32_t lower_coarse_integration = coarse_integration - 1;
    int32_t lower_fine_integration = fine_integration_max;

    // Check if lower case is invalid (take upper coarse)
    if (lower_coarse_integration < coarse_integration_min) {
      coarse_integration = upper_coarse_integration;
      fine_integration = upper_fine_integration;
    }
    // Check if upper case is invalid (take lower coarse)
    else if (upper_coarse_integration > coarse_integration_max) {
      coarse_integration = lower_coarse_integration;
      fine_integration = lower_fine_integration;
    }
    // Both are good
    else {
      // Calculate error to decide which is better
      int32_t upper_error = abs((mt->line_length * upper_coarse_integration + upper_fine_integration) - integration);
      int32_t lower_error = abs((mt->line_length * lower_coarse_integration + lower_fine_integration) - integration);

      if (upper_error < lower_error) {
        coarse_integration = upper_coarse_integration;
        fine_integration = upper_fine_integration;
      } else {
        coarse_integration = lower_coarse_integration;
        fine_integration = lower_fine_integration;
      }
    }
  }

  /* Fix saturations */
  Bound(fine_integration, fine_integration_min, fine_integration_max);
  Bound(coarse_integration, coarse_integration_min, coarse_integration_max);

  /* Set the registers */
  mt->real_exposure = (float)(coarse_integration * mt->line_length + fine_integration) / (mt->vt_pix_clk * 1000);
  write_reg(mt, MT9F002_COARSE_INTEGRATION_TIME, coarse_integration, 2);
  write_reg(mt, MT9F002_FINE_INTEGRATION_TIME_, fine_integration, 2);
}

/**
 *  Calculate the gain based on value of 1.0 -> 63.50
 */
static uint16_t mt9f002_calc_gain(float gain)
{
  // Check if gain is valid
  if (gain < 1.0) {
    gain = 1.0;
  }

  // Calculation of colamp, analg3 and digital gain based on table 19 p56
  uint8_t colamp_gain, analog_gain3, digital_gain;
  if (gain < 1.50) {
    // This is not recommended
    colamp_gain = 0;
    analog_gain3 = 0;
    digital_gain = 1;
  } else if (gain < 3.0) {
    colamp_gain = 1;
    analog_gain3 = 0;
    digital_gain = 1;
  } else if (gain < 6.0) {
    colamp_gain = 2;
    analog_gain3 = 0;
    digital_gain = 1;
  } else if (gain < 16.0) {
    colamp_gain = 3;
    analog_gain3 = 0;
    digital_gain = 1;
  } else if (gain < 32.0) {
    colamp_gain = 3;
    analog_gain3 = 0;
    digital_gain = 2;
  } else {
    colamp_gain = 3;
    analog_gain3 = 0;
    digital_gain = 4;
  }

  // Calculate gain 2 (fine gain)
  uint16_t analog_gain2 = gain / (float)digital_gain / (float)(1 << colamp_gain) / (float)(1 << analog_gain3) * 64.0;
  Bound(analog_gain2, 1, 127);

  return (analog_gain2 & 0x7F) | ((analog_gain3 & 0x7) << 7) | ((colamp_gain & 0x3) << 10) | ((digital_gain & 0xF) << 12);
}

/**
 * Sets the GreenR, Blue, Red and GreenB gains
 */
void mt9f002_set_gains(struct mt9f002_t *mt)
{
  write_reg(mt, MT9F002_GREEN1_GAIN, mt9f002_calc_gain(mt->gain_green1), 2);
  write_reg(mt, MT9F002_BLUE_GAIN,   mt9f002_calc_gain(mt->gain_blue), 2);
  write_reg(mt, MT9F002_RED_GAIN,    mt9f002_calc_gain(mt->gain_red), 2);
  write_reg(mt, MT9F002_GREEN2_GAIN, mt9f002_calc_gain(mt->gain_green2), 2);
}

void mt9f002_calc_resolution(struct mt9f002_t *mt)
{
  struct v4l2_rect rect;
  struct v4l2_rect crop;
  unsigned int x_odd_inc;
  unsigned int y_odd_inc;
  unsigned int hratio;
  unsigned int vratio;
  unsigned int width;
  unsigned int height;
  unsigned int ratio;
  unsigned int xMultiple;
  unsigned int div_res;
  static const uint8_t xy_odd_inc_tab[] = {1, 1, 3, 3, 7, 7, 7, 7, 15};

  crop.left   = mt->offset_x;
  crop.top    = mt->offset_y;
  crop.width  = mt->sensor_width;
  crop.height = mt->sensor_height;

  VERBOSE_PRINT("Requested output    - width: %i, height: %i\n", mt->output_width, mt->output_height);
  VERBOSE_PRINT("Requested crop      - top: %i, left: %i, width: %i, height: %i\n", crop.top, crop.left, crop.width,
                crop.height);

  /* Clamp the crop rectangle boundaries and align them to a multiple of 2
   * pixels to ensure a GRBG Bayer pattern.
   */
  rect.left = clamp(ALIGN(crop.left, 2),
                    CFG_MT9F002_X_ADDR_MIN,
                    CFG_MT9F002_X_ADDR_MAX);
  rect.top = clamp(ALIGN(crop.top, 2),
                   CFG_MT9F002_Y_ADDR_MIN,
                   CFG_MT9F002_Y_ADDR_MAX);
  rect.width = clamp(ALIGN(crop.width, 4), 1,
                     CFG_MT9F002_PIXEL_ARRAY_WIDTH);
  rect.height = clamp(ALIGN(crop.height, 4), 1,
                      CFG_MT9F002_PIXEL_ARRAY_HEIGHT);
  rect.width = min(rect.width, CFG_MT9F002_PIXEL_ARRAY_WIDTH - rect.left);
  rect.height = min(rect.height, CFG_MT9F002_PIXEL_ARRAY_HEIGHT - rect.top);

  /* Clamp the width and height to avoid dividing by zero. */
  width = clamp_t(unsigned int, ALIGN(mt->output_width, 2),
                  max(rect.width / 8, (__s32) CFG_MT9F002_WINDOW_WIDTH_MIN),
                  rect.width);
  height = clamp_t(unsigned int, ALIGN(mt->output_height, 2),
                   max((rect.height / 8), (__s32) CFG_MT9F002_WINDOW_HEIGHT_MIN),
                   rect.height);
  /* Calculate binning / skipping for X */
  div_res = rect.width / width;
  div_res = clamp_t(unsigned int, div_res, 1U, 4U);
  x_odd_inc = xy_odd_inc_tab[div_res];
  /* Calculate binning / skipping for Y */
  div_res = rect.height / height;
  div_res = clamp_t(unsigned int, div_res, 1U, 4U);
  y_odd_inc = xy_odd_inc_tab[div_res];
  /* Align left offset to 8 */
  xMultiple = 8 * ((x_odd_inc + 1) / 2);
  if (rect.left % xMultiple) {
    rect.left -= (rect.left % xMultiple);
  }
  /* Calculate remaining scaling not handled by binning / skipping */
  hratio = ((rect.width / ((x_odd_inc + 1) / 2) * MT9F002_SCALER_N) + width - 1) /
           width;
  vratio = ((rect.height / ((y_odd_inc + 1) / 2) * MT9F002_SCALER_N) + height -
            1) / height;
  ratio = min(hratio, vratio);
  /* Check ratio */
  if (ratio > CFG_SCALER_M_MAX) {
    /* Fix ratio to maximum and adjust the crop window */
    ratio = CFG_SCALER_M_MAX;
    rect.width = mt->output_width * ratio * ((x_odd_inc + 1) / 2) /
                 MT9F002_SCALER_N;
    rect.height = mt->output_height * ratio * ((y_odd_inc + 1) / 2) /
                  MT9F002_SCALER_N;
  }
  VERBOSE_PRINT("Calculated skipping - x: %i, y: %i\n", x_odd_inc, y_odd_inc);
  VERBOSE_PRINT("Calculated scaler   - %i/%i = %0.3f\n", MT9F002_SCALER_N, ratio,
                ((float) MT9F002_SCALER_N) / ((float) ratio));
  /* Update crop */
  crop = rect;
  VERBOSE_PRINT("Granted crop        - top: %i, left: %i, width: %i, height: %i\n", crop.top, crop.left, crop.width,
                crop.height);
  VERBOSE_PRINT("Granted output      - width: %i, height: %i\n", width, height);

  /* Check if scaling configuration has changed */
  if (mt->x_odd_inc != x_odd_inc ||
      mt->y_odd_inc != y_odd_inc || mt->output_scaler != (float)MT9F002_SCALER_N / ((float) ratio)) {
    /* Update values */
    mt->output_scaler = (float)MT9F002_SCALER_N / ((float) ratio);
    mt->x_odd_inc = x_odd_inc;
    mt->y_odd_inc = y_odd_inc;
  }
  mt->offset_x = crop.left;
  mt->offset_y = crop.top;
  mt->sensor_width = crop.width;
  mt->sensor_height = crop.height;
}

void mt9f002_set_resolution(struct mt9f002_t *mt)
{
  /* Set window pos */
  write_reg(mt, MT9F002_X_ADDR_START, mt->offset_x, 2);
  write_reg(mt, MT9F002_Y_ADDR_START, mt->offset_y, 2);
  write_reg(mt, MT9F002_X_ADDR_END, mt->offset_x + mt->sensor_width - 1, 2);
  write_reg(mt, MT9F002_Y_ADDR_END, mt->offset_y + mt->sensor_height - 1, 2);
  /* Set output resolution */
  write_reg(mt, MT9F002_X_OUTPUT_SIZE, mt->output_width, 2);
  write_reg(mt, MT9F002_Y_OUTPUT_SIZE, mt->output_height, 2);
  /* scaler */
  if (mt->output_scaler != 1) {
    /* enable scaling mode */
    write_reg(mt, MT9F002_SCALING_MODE, 2, 2);
    write_reg(mt, MT9F002_SCALE_M, (int) ceil((float)MT9F002_SCALER_N / mt->output_scaler), 2);
    VERBOSE_PRINT("SCALE_M: %i\n", (int) ceil((float)MT9F002_SCALER_N / mt->output_scaler));
  }
  return;
}

/**
 * Initialisation of the Aptina MT9F002 CMOS sensor
 * (front camera)
 */
void mt9f002_init(struct mt9f002_t *mt)
{
  /* Reset the device */
  //TODO???

  /* Setup i2c transaction */
  mt->i2c_trans.status = I2CTransDone;

  /* Software reset */
  write_reg(mt, MT9F002_SOFTWARE_RESET, 0x1, 1);
  usleep(1000000); // Wait for one second

  /* Based on the interface configure stage 1 */
  if (mt->interface == MT9F002_MIPI || mt->interface == MT9F002_HiSPi) {
    mt9f002_mipi_stage1(mt);
  } else {
    mt9f002_parallel_stage1(mt);
  }

  /* Set the PLL based on Input clock and wanted clock frequency */
  mt9f002_set_pll(mt);

  /* Calculate binning/skipping/scaling for requested sensor domain and output resolution */
  mt9f002_calc_resolution(mt);
  /* Update blanking timings based on chosen binning/skipping/scaling */
  mt9f002_blanking_init(mt);
  /* Based on the interface configure stage 2 */
  if (mt->interface == MT9F002_MIPI || mt->interface == MT9F002_HiSPi) {
    mt9f002_mipi_stage2(mt);
  } else {
    mt9f002_parallel_stage2(mt);
  }

  mt9f002_set_resolution(mt);

  /* Update blanking (based on FPS) */
  mt9f002_set_blanking(mt);
  isp_request_statistics_yuv_window(0 , (uint16_t) mt->sensor_width, 0, mt->sensor_height, mt->x_odd_inc, mt->y_odd_inc);
  /* Update exposure (based on target_exposure) */
  mt9f002_set_exposure(mt);

  /* Update gains for the different pixel colors */
  mt9f002_set_gains(mt);

  /* Based on the interface configure stage 3 */
  if (mt->interface == MT9F002_MIPI || mt->interface == MT9F002_HiSPi) {
    mt9f002_mipi_stage3(mt);
  }

  /* Turn the stream on */
  write_reg(mt, MT9F002_MODE_SELECT, 0x01, 1);
}

void mt9f002_update_resolution(struct mt9f002_t *mt)
{
  /* Calculate binning/skipping/scaling for requested sensor domain and output resolution */
  mt9f002_calc_resolution(mt);
  /* Update blanking timings based on chosen binning/skipping/scaling */
  mt9f002_blanking_init(mt);
  mt9f002_set_resolution(mt);
  /* Update blanking (based on FPS) */
  mt9f002_set_blanking(mt);
  // Update the isp_config
  //double xScaler = mt->output_scaler * 2.0/(mt->x_odd_inc + 1);
  //double yScaler = mt->output_scaler * 2.0/(mt->y_odd_inc + 1);
  //isp_request_statistics_yuv_window(ALIGN(mt->offset_x,16), ALIGN(mt->offset_x + mt->sensor_width,16), ALIGN(mt->offset_y,16), ALIGN(mt->offset_y + mt->sensor_height,16));
  isp_request_statistics_yuv_window(0 , (uint16_t) mt->sensor_width, 0, mt->sensor_height, mt->x_odd_inc, mt->y_odd_inc);
}
