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
#include "isp/libisp.h"
#include "math/pprz_algebra_int.h"
#include "peripherals/video_device.h"

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/videodev2.h>

#include "generated/airframe.h"
#ifdef BOARD_DISCO
#include "boards/disco.h"
#else
#include "boards/bebop.h"
#endif

#define PRINT(string,...) fprintf(stderr, "[MT9F002->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

#if MT9F002_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

// The sequencing of the pixel array is controlled by the x_addr_start, y_addr_start,
// x_addr_end, and y_addr_end registers. For both parallel and serial HiSPi interfaces, the
// output image size is controlled by the x_output_size and y_output_size registers.

// Horizontal Mirror
// Vertical Flip

/** Exposure of the front camera of the bebop. Experimental values:
 * Outside: 15
 * Inside well lit: 30
 * Inside poorly lit: 60
 */
#ifndef MT9F002_TARGET_EXPOSURE
#define MT9F002_TARGET_EXPOSURE 30
#endif

/* Set the colour balance gains */
#ifndef MT9F002_GAIN_GREEN1
#define MT9F002_GAIN_GREEN1 2.0
#endif

#ifndef MT9F002_GAIN_GREEN2
#define MT9F002_GAIN_GREEN2 2.0
#endif

#ifndef MT9F002_GAIN_RED
#define MT9F002_GAIN_RED 1.4
#endif

#ifndef MT9F002_GAIN_BLUE
#define MT9F002_GAIN_BLUE 2.7
#endif

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
  .fps = MT9F002_TARGET_FPS,
  .camera_intrinsics = {
    .focal_x = MT9F002_FOCAL_X,
    .focal_y = MT9F002_FOCAL_Y,
    .center_x = MT9F002_CENTER_X,
    .center_y = MT9F002_CENTER_Y,
    .Dhane_k = MT9F002_DHANE_K,
  }
};

/* Initialize MT9F002 chipset (Front camera) */
struct mt9f002_t mt9f002 = {
  // Precomputed values to go from InputCLK of (26/2)MHz to 96MH
  .interface = MT9F002_PARALLEL,
  .input_clk_freq = (26 / 2),
  .vt_pix_clk_div = 7,
  .vt_sys_clk_div = 1,
  .pre_pll_clk_div = 1,
  .pll_multiplier = 59,
  .op_pix_clk_div = 8,
  .op_sys_clk_div = 1,
  .shift_vt_pix_clk_div = 1,
  .rowSpeed_2_0 = 1,
  .row_speed_10_8 = 1,

  // Initial values
  .target_fps = MT9F002_TARGET_FPS,
  .target_exposure = MT9F002_TARGET_EXPOSURE,
  .gain_green1 = MT9F002_GAIN_GREEN1,
  .gain_blue = MT9F002_GAIN_BLUE,
  .gain_red = MT9F002_GAIN_RED,
  .gain_green2 = MT9F002_GAIN_GREEN2,
  .output_width = MT9F002_OUTPUT_WIDTH,
  .output_height = MT9F002_OUTPUT_HEIGHT,
  .output_scaler = 1,
  .offset_x = Max(0, (CFG_MT9F002_PIXEL_ARRAY_WIDTH - CFG_MT9F002_PIXEL_ARRAY_WIDTH / MT9F002_ZOOM) / 2
                  + MT9F002_OFFSET_X * CFG_MT9F002_PIXEL_ARRAY_WIDTH),
  .offset_y = Max(0, (CFG_MT9F002_PIXEL_ARRAY_HEIGHT - CFG_MT9F002_PIXEL_ARRAY_HEIGHT / MT9F002_ZOOM) / 2
                  + MT9F002_OFFSET_Y * CFG_MT9F002_PIXEL_ARRAY_HEIGHT),
  .sensor_width = CFG_MT9F002_PIXEL_ARRAY_WIDTH / MT9F002_ZOOM,
  .sensor_height = CFG_MT9F002_PIXEL_ARRAY_HEIGHT / MT9F002_ZOOM,
  .x_odd_inc = 1,
  .y_odd_inc = 1,

  // I2C connection port
  .i2c_periph = &i2c0,

  // settings
  .set_zoom = MT9F002_ZOOM,
  .set_offset_x = MT9F002_OFFSET_X,
  .set_offset_y = MT9F002_OFFSET_Y,
};

struct blanking_t {
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

  // Fix signdness based on length
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
    PRINT("Write_reg with incorrect length %d\r\n", len);
  }

  // Transmit the buffer
  i2c_blocking_transmit(mt->i2c_periph, &mt->i2c_trans, MT9F002_ADDRESS, len + 2);
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
  i2c_blocking_transceive(mt->i2c_periph, &mt->i2c_trans, MT9F002_ADDRESS, 2, len);

  /* Fix signdness */
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
  write_reg(mt, MT9F002_EXTRA_DELAY, 0x0000, 2);
  write_reg(mt, MT9F002_RESET_REGISTER, 0x0118, 2);
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
  //write_reg(mt, MT9F002_RESET_REGISTER       , 0x14C8, 2); // reset bad frame
  write_reg(mt, MT9F002_COARSE_INTEGRATION_TIME, 0x08C3, 2);
  write_reg(mt, MT9F002_DIGITAL_TEST, 0x0000   , 2);
  //write_reg(mt, MT9F002_DATAPATH_SELECT      , 0xd881, 2); // permanent line valid
  write_reg(mt, MT9F002_DATAPATH_SELECT, 0xd880, 2);
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

// set the frame rate of the camera
static void mt9f002_set_blanking(struct mt9f002_t *mt)
{
  //Set the blanking configuration
  if (mt->x_odd_inc > 1) {
    if (mt->y_odd_inc > 1) {
      /* Binning XY */
      mt9f002_blanking.min_line_blanking_pck = 2950;
      mt9f002_blanking.min_line_length_pck = 4650;
      mt9f002_blanking.min_line_fifo_pck = 120;
      mt9f002_blanking.fine_integration_time_max_margin = 2000;
      mt9f002_blanking.fine_integration_time_min = 2200;
    } else {
      /* Binning X */
      mt9f002_blanking.min_line_blanking_pck = 0;
      mt9f002_blanking.min_line_length_pck = 3495;
      mt9f002_blanking.min_line_fifo_pck = 60;
      mt9f002_blanking.fine_integration_time_max_margin = 1500;
      mt9f002_blanking.fine_integration_time_min = 1900;
    }
  } else {
    mt9f002_blanking.fine_integration_time_max_margin = 1316;
    mt9f002_blanking.fine_integration_time_min = 1032;
    mt9f002_blanking.min_line_fifo_pck = 60;
    if (mt->output_scaler > 1) {
      /* Scaler mode */
      mt9f002_blanking.min_line_blanking_pck = 2400;
      mt9f002_blanking.min_line_length_pck = 1750;
    } else {
      /* Normal mode */
      mt9f002_blanking.min_line_blanking_pck = 1316;
      mt9f002_blanking.min_line_length_pck = 1032;
    }
  }

  uint16_t x_addr_start = read_reg(mt, MT9F002_X_ADDR_START, 2);
  uint16_t x_addr_end   = read_reg(mt, MT9F002_X_ADDR_END, 2);

  float subsamplingX_factor = (float)(1 + mt->x_odd_inc) / 2.0f; // See page 52
  float subsamplingY_factor = (float)(1 + mt->y_odd_inc) / 2.0f; // See page 52

  /* Calculate minimum line length based on p. 53 */
  // line length based on window width
  uint16_t min_line_length = (uint16_t)((x_addr_end - x_addr_start + mt->x_odd_inc) / subsamplingX_factor / 2) +
                             mt9f002_blanking.min_line_blanking_pck;

  // must be strictly longer than min length set in the min_line_length_pck register
  min_line_length = Max(min_line_length, mt9f002_blanking.min_line_length_pck + 1);

  // row time must be strictly larger than the time needed for the FIFO to output the data
  if (mt->interface == MT9F002_MIPI || mt->interface == MT9F002_HiSPi) {
    // 2 lanes, pll clocks
    min_line_length = Max(min_line_length,
                          (mt->output_width * mt->vt_pix_clk / mt->op_pix_clk / 2) + mt9f002_blanking.min_line_fifo_pck + 1);
  } else {
    // pll clocks
    min_line_length = Max(min_line_length,
                          (mt->output_width * mt->vt_pix_clk / mt->op_pix_clk) + mt9f002_blanking.min_line_fifo_pck + 1);
  }

  /* Do some magic to get it to work with P7 ISP (with horizontal blanking) */
  uint32_t clkRatio_num = mt->op_sys_clk_div * mt->op_pix_clk_div * mt->row_speed_10_8 * (1 + mt->shift_vt_pix_clk_div);
  uint32_t clkRatio_den = mt->vt_sys_clk_div * mt->vt_pix_clk_div;

  /* Divide by the GCD to find smallest ratio */
  uint32_t clkRatio_gcd = int32_gcd(clkRatio_num, clkRatio_den);
  clkRatio_num = clkRatio_num / clkRatio_gcd;
  clkRatio_den = clkRatio_den / clkRatio_gcd;

  /* Calculate minimum horizontal blanking, since fpga line_length must be divisible by 2 */
  uint32_t min_horizontal_blanking = clkRatio_num;
  if ((clkRatio_den % 2) != 0) {
    min_horizontal_blanking *= 2;
  }

  /* Fix fpga correction based on min horizontal blanking */
  if ((min_line_length % min_horizontal_blanking) != 0) {
    min_line_length += min_horizontal_blanking - (min_line_length % min_horizontal_blanking);
  }
  mt->line_length = min_line_length;

  /* Calculate minimum frame length lines */
  uint16_t y_addr_start = read_reg(mt, MT9F002_Y_ADDR_START, 2);
  uint16_t y_addr_end   = read_reg(mt, MT9F002_Y_ADDR_END, 2);
  uint16_t min_frame_blanking_lines = read_reg(mt, MT9F002_MIN_FRAME_BLANKING_LINES, 2);
  // frame time is limited by total number of rows (EQ 10)
  uint16_t min_frame_length = (y_addr_end - y_addr_start + 1) / subsamplingY_factor + min_frame_blanking_lines;
  mt->frame_length = min_frame_length;

  /* Calculate FPS we get using these minimums (Maximum FPS) */
  mt->real_fps = mt->vt_pix_clk * 1e6 / (float)(mt->line_length * mt->frame_length);
  VERBOSE_PRINT("Maximum FPS: %0.3f\n", mt->real_fps);

  /* Check if we need to downscale the FPS and bruteforce better solution */
  if (mt->target_fps > 0 && mt->target_fps < mt->real_fps) {
    float min_fps_err = mt->real_fps - mt->target_fps;
    float new_fps = mt->real_fps;

    // Go through all possible line lengths
    for (uint32_t ll = min_line_length; ll <= MT9F002_LINE_LENGTH_MAX; ll += min_horizontal_blanking) {
      // Go through all possible frame lengths
      for (uint32_t fl = min_frame_length; fl < MT9F002_FRAME_LENGTH_MAX; fl++) {
        new_fps = mt->vt_pix_clk * 1000000 / (float)(ll * fl);

        // Calculate FPS error and save if it is better
        float fps_err = new_fps - mt->target_fps;

        // Stop searching if FPS is lower or equal
        if (fps_err < 0) {
          break;
        }

        if (fps_err < min_fps_err) {
          min_fps_err = fps_err;
          mt->line_length   = ll;
          mt->frame_length  = fl;
          mt->real_fps      = new_fps;
        }
      }

      // Calculate if next step is still needed (since we only need to go one step below target_fps)
      new_fps = mt->vt_pix_clk * 1000000 / (float)(ll * min_frame_length);

      // Stop searching if FPS is lower or equal
      if (new_fps - mt->target_fps <= 0.f) {
        break;
      }
    }
  }

  VERBOSE_PRINT("Set FPS: %0.3f\n", mt->vt_pix_clk * 1000000 / (float)(mt->line_length * mt->frame_length));

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
      int32_t upper_error = abs((mt->line_length * upper_coarse_integration + upper_fine_integration) - (int32_t)integration);
      int32_t lower_error = abs((mt->line_length * lower_coarse_integration + lower_fine_integration) - (int32_t)integration);

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

// settings to update resolution, color and exposure
float mt9f002_send_resolution;
float mt9f002_send_color;
float mt9f002_send_exposure;


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

static void mt9f002_calc_resolution(struct mt9f002_t *mt)
{
  struct v4l2_rect crop, rect;
  unsigned int x_odd_inc, y_odd_inc;
  unsigned int hratio, vratio;
  unsigned int width, height;
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

  /* Clamp the crop rectangle boundaries and Align them to a multiple of 2
   * pixels to ensure a GRBG Bayer pattern.
   */
  rect.width = Clip(Align(crop.width, 4), 1, CFG_MT9F002_PIXEL_ARRAY_WIDTH);
  rect.height = Clip(Align(crop.height, 4), 1, CFG_MT9F002_PIXEL_ARRAY_HEIGHT);

  /* Clamp the width and height to avoid dividing by zero. */
  width = Clip(Align(mt->output_width, 2), Max((int32_t)rect.width / 8, CFG_MT9F002_WINDOW_WIDTH_MIN), (int32_t)rect.width);
  height = Clip(Align(mt->output_height, 2), Max((int32_t)rect.height / 8, CFG_MT9F002_WINDOW_HEIGHT_MIN), (int32_t)rect.height);

  /* Calculate binning / skipping, we enforce that binning in X and Y are the same */
  div_res = Min(rect.width / width, rect.height / height);
  div_res = Clip(div_res, 1, 4);
  x_odd_inc = xy_odd_inc_tab[div_res];
  y_odd_inc = xy_odd_inc_tab[div_res];

  /* Calculate remaining scaling not handled by binning / skipping */
  hratio = (rect.width / ((x_odd_inc + 1) / 2) * MT9F002_SCALER_N) / width;
  vratio = (rect.height / ((y_odd_inc + 1) / 2) * MT9F002_SCALER_N) / height;
  ratio = Min(hratio, vratio);

  /* Check ratio */
  if (ratio > CFG_SCALER_M_MAX) {
    /* Fix ratio to maximum and adjust the crop window */
    ratio = CFG_SCALER_M_MAX;
  }

  rect.width = mt->output_width * ratio * ((x_odd_inc + 1) / 2) / MT9F002_SCALER_N;
  rect.height = mt->output_height * ratio * ((y_odd_inc + 1) / 2) / MT9F002_SCALER_N;

  VERBOSE_PRINT("Calculated skipping - x: %i, y: %i\n", x_odd_inc, y_odd_inc);
  VERBOSE_PRINT("Calculated scaler   - %i/%i = %0.3f\n", MT9F002_SCALER_N, ratio, MT9F002_SCALER_N / ((float)ratio));

  // center crop to same as requested
  rect.left = crop.left + ((int32_t)crop.width - (int32_t)rect.width) / 2;
  rect.top = crop.top + ((int32_t)crop.height - (int32_t)rect.height) / 2;

  rect.left = Min(rect.left, CFG_MT9F002_PIXEL_ARRAY_WIDTH - (int32_t)rect.width);
  rect.top = Min(rect.top, CFG_MT9F002_PIXEL_ARRAY_HEIGHT - (int32_t)rect.height);
  rect.left = Clip(Align(rect.left, 2), CFG_MT9F002_X_ADDR_MIN, CFG_MT9F002_X_ADDR_MAX);
  rect.top = Clip(Align(rect.top, 2), CFG_MT9F002_Y_ADDR_MIN, CFG_MT9F002_Y_ADDR_MAX);

  /* Align left offset to 8 */
  xMultiple = 8 * ((x_odd_inc + 1) / 2);
  rect.left = Align(rect.left, (int32_t)xMultiple);

  /* Align top offset to 2 */
  rect.top = Align(rect.top, 4);

  /* Update crop */
  crop = rect;
  VERBOSE_PRINT("Granted crop    - top: %i, left: %i, width: %i, height: %i\n", crop.top, crop.left, crop.width,
                crop.height);
  VERBOSE_PRINT("Granted output  - width: %i, height: %i\n", width, height);

  /* Update values */
  mt->output_scaler = ratio;
  mt->x_odd_inc = x_odd_inc;
  mt->y_odd_inc = y_odd_inc;

  mt->offset_x = crop.left;
  mt->offset_y = crop.top;
  mt->sensor_width = crop.width;
  mt->sensor_height = crop.height;
}

static void mt9f002_set_resolution(struct mt9f002_t *mt)
{
  /* Set window pos */
  write_reg(mt, MT9F002_X_ADDR_START, Max(mt->offset_x, CFG_MT9F002_X_ADDR_MIN), 2);
  write_reg(mt, MT9F002_Y_ADDR_START, Max(mt->offset_y, CFG_MT9F002_Y_ADDR_MIN), 2);
  write_reg(mt, MT9F002_X_ADDR_END, Min(mt->offset_x + mt->sensor_width - 1, CFG_MT9F002_X_ADDR_MAX), 2);
  write_reg(mt, MT9F002_Y_ADDR_END, Min(mt->offset_y + mt->sensor_height - 1, CFG_MT9F002_Y_ADDR_MAX), 2);

  /* Set output resolution */
  write_reg(mt, MT9F002_X_OUTPUT_SIZE, mt->output_width, 2);
  write_reg(mt, MT9F002_Y_OUTPUT_SIZE, mt->output_height, 2);

  /* scaler */
  if (mt->output_scaler > 1) {
    /* enable scaling mode */
    write_reg(mt, MT9F002_SCALING_MODE, 2, 2);
    write_reg(mt, MT9F002_DATAPATH_SELECT, 0xd8b0, 2);  // bayer resampling
    write_reg(mt, MT9F002_SCALE_M, mt->output_scaler, 2);
  }

  /* Binning / Skipping */
  if (mt->x_odd_inc > 1 || mt->x_odd_inc > 1) {
    write_reg(mt, MT9F002_READ_MODE, 0x0441, 2);
  } else {
    write_reg(mt, MT9F002_READ_MODE, 0x0041, 2);
  }
  write_reg(mt, MT9F002_X_ODD_INC, mt->x_odd_inc, 2);
  write_reg(mt, MT9F002_Y_ODD_INC, mt->y_odd_inc, 2);

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

  /* Based on the interface configure stage 2 */
  if (mt->interface == MT9F002_MIPI || mt->interface == MT9F002_HiSPi) {
    mt9f002_mipi_stage2(mt);
  } else {
    mt9f002_parallel_stage2(mt);
  }

  mt9f002_set_resolution(mt);

  /* Update blanking (based on FPS) */
  mt9f002_set_blanking(mt);

  /* Update statistics window (inside of cropped sensor area)*/
  isp_request_statistics_yuv_window(0, mt->sensor_width, 0, mt->sensor_height, 1, 1);

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

  VERBOSE_PRINT("MT9F002 initialized\n");
}

void mt9f002_reset_color(struct mt9f002_t *mt)
{
  mt->gain_red    = MT9F002_GAIN_RED;
  mt->gain_blue   = MT9F002_GAIN_BLUE;
  mt->gain_green1 = MT9F002_GAIN_GREEN1;
  mt->gain_green2 = MT9F002_GAIN_GREEN2;
  mt9f002_set_gains(mt);
}

void mt9f002_reset_exposure(struct mt9f002_t *mt)
{
  mt->target_exposure = MT9F002_TARGET_EXPOSURE;
  mt9f002_set_exposure(mt);
}

/* Handler for propagating user resolution change so the camera
 *
 */
void mt9f002_setting_update_resolution(float in UNUSED)
{
  mt9f002.sensor_width = CFG_MT9F002_PIXEL_ARRAY_WIDTH / mt9f002.set_zoom;
  mt9f002.sensor_height = CFG_MT9F002_PIXEL_ARRAY_HEIGHT / mt9f002.set_zoom;

  mt9f002.offset_x = Max(0, (CFG_MT9F002_PIXEL_ARRAY_WIDTH - CFG_MT9F002_PIXEL_ARRAY_WIDTH / mt9f002.set_zoom) / 2
                         + mt9f002.set_offset_x * CFG_MT9F002_PIXEL_ARRAY_WIDTH);
  mt9f002.offset_y = Max(0, (CFG_MT9F002_PIXEL_ARRAY_HEIGHT - CFG_MT9F002_PIXEL_ARRAY_HEIGHT / mt9f002.set_zoom) / 2
                         + mt9f002.set_offset_y * CFG_MT9F002_PIXEL_ARRAY_HEIGHT);

  /* Calculate binning/skipping/scaling for requested sensor domain and output resolution */
  mt9f002_calc_resolution(&mt9f002);

  mt9f002_set_resolution(&mt9f002);

  /* Update blanking (based on FPS) */
  mt9f002_set_blanking(&mt9f002);

  // Update the isp_config
  isp_request_statistics_yuv_window(0, mt9f002.sensor_width, 0, mt9f002.sensor_height, 1, 1);
}

void mt9f002_setting_update_color(float in UNUSED)
{
  mt9f002_set_gains(&mt9f002);
}

void mt9f002_setting_update_exposure(float in UNUSED)
{
  mt9f002_set_exposure(&mt9f002);
}

