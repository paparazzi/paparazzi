/*
 * Copyright (C) Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi
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
* @file boards/bebop/mt9f002.h
*
* Initialization and configuration of the MT9F002 CMOS Chip
*/

#ifndef MT9F002H
#define MT9F002H

#include "std.h"
#include "mcu_periph/i2c.h"
#include "generated/airframe.h"

#define CFG_SCALER_M_MIN 16
#define CFG_SCALER_M_MAX 128
#define CFG_MT9F002_WINDOW_WIDTH_MIN 1
#define CFG_MT9F002_WINDOW_HEIGHT_MIN 1
#define CFG_MT9F002_PIXEL_ARRAY_HEIGHT 3288
#define CFG_MT9F002_PIXEL_ARRAY_WIDTH 4608
#define CFG_MT9F002_X_ADDR_MIN 24
#define CFG_MT9F002_X_ADDR_MAX 4647
#define CFG_MT9F002_Y_ADDR_MIN 0
#define CFG_MT9F002_Y_ADDR_MAX CFG_MT9F002_PIXEL_ARRAY_HEIGHT

#define RES_VGA        0
#define RES_720p       1
#define RES_720p_4_3   2
#define RES_1080p      3
#define RES_1080p_4_3  4
#define RES_FULL       5

#ifdef MT9F002_RESOLUTION
#if MT9F002_RESOLUTION == RES_VGA
#define MT9F002_OUTPUT_WIDTH 640
#define MT9F002_OUTPUT_HEIGHT 480
#elif MT9F002_RESOLUTION == RES_720p
#define MT9F002_OUTPUT_WIDTH 1280
#define MT9F002_OUTPUT_HEIGHT 720
#elif MT9F002_RESOLUTION == RES_720p_4_3
#define MT9F002_OUTPUT_WIDTH 960
#define MT9F002_OUTPUT_HEIGHT 720
#elif MT9F002_RESOLUTION == RES_1080p
#define MT9F002_OUTPUT_WIDTH 1920
#define MT9F002_OUTPUT_HEIGHT 1080
#elif MT9F002_RESOLUTION == RES_1080p_4_3
#define MT9F002_OUTPUT_WIDTH 1440
#define MT9F002_OUTPUT_HEIGHT 1080
#elif MT9F002_RESOLUTION == RES_FULL
// Doesn't work with isp
//#define MT9F002_OUTPUT_WIDTH 4384
//#define MT9F002_OUTPUT_HEIGHT 3288
#define MT9F002_OUTPUT_WIDTH 2048
#define MT9F002_OUTPUT_HEIGHT 2048
#else // default MT9F002_RESOLUTION
#define MT9F002_OUTPUT_WIDTH 640
#define MT9F002_OUTPUT_HEIGHT 640
#endif

#else // MT9F002_RESOLUTION

#ifndef MT9F002_OUTPUT_WIDTH
#define MT9F002_OUTPUT_WIDTH 640
#endif
#ifndef MT9F002_OUTPUT_HEIGHT
#define MT9F002_OUTPUT_HEIGHT 640
#endif
#endif

// Signed fractional offset from centre of image of original sensor [-0.5,0.5]
#ifndef MT9F002_OFFSET_X
#define MT9F002_OFFSET_X 0.
#endif

// Signed fractional offset from centre of image of original sensor [-0.5,0.5]
#ifndef MT9F002_OFFSET_Y
#define MT9F002_OFFSET_Y 0.
#endif

// Zoom factor of image
#ifndef MT9F002_ZOOM
#define MT9F002_ZOOM 1.
#endif

#ifndef MT9F002_TARGET_FPS
#define MT9F002_TARGET_FPS 30
#endif

// parameters for undistortion, defaults are rough estimates
#ifndef MT9F002_FOCAL_X
#define MT9F002_FOCAL_X (MT9F002_ZOOM * MT9F002_OUTPUT_WIDTH / 2.f)
#endif
#ifndef MT9F002_FOCAL_Y
#define MT9F002_FOCAL_Y (MT9F002_ZOOM * MT9F002_OUTPUT_HEIGHT / 2.f)
#endif
#ifndef MT9F002_CENTER_X
#define MT9F002_CENTER_X (MT9F002_OUTPUT_WIDTH * (.5f - MT9F002_ZOOM * MT9F002_OFFSET_X))
#endif
#ifndef MT9F002_CENTER_Y
#define MT9F002_CENTER_Y (MT9F002_OUTPUT_HEIGHT * (.5f - MT9F002_ZOOM * MT9F002_OFFSET_Y))
#endif
#ifndef MT9F002_DHANE_K
#define MT9F002_DHANE_K 1.25f
#endif

/* Interface types for the MT9F002 connection */
enum mt9f002_interface {
  MT9F002_MIPI,     ///< MIPI type connection
  MT9F002_HiSPi,    ///< HiSPi type connection
  MT9F002_PARALLEL  ///< Parallel type connection
};

/* Main configuration structure */
struct mt9f002_t {
  enum mt9f002_interface interface;   ///< Interface used to connect
  float input_clk_freq;               ///< Input clock frequency
  uint16_t vt_pix_clk_div;            ///< Fixed PLL config from calculator tool
  uint16_t vt_sys_clk_div;            ///< Fixed PLL config from calculator tool
  uint16_t pre_pll_clk_div;           ///< Fixed PLL config from calculator tool
  uint16_t pll_multiplier;            ///< Fixed PLL config from calculator tool
  uint16_t op_pix_clk_div;            ///< Fixed PLL config from calculator tool
  uint16_t op_sys_clk_div;            ///< Fixed PLL config from calculator tool
  uint8_t shift_vt_pix_clk_div;       ///< Fixed PLL config from calculator tool
  uint8_t rowSpeed_2_0;               ///< Fixed PLL config from calculator tool
  uint8_t row_speed_10_8;             ///< Fixed PLL config from calculator tool
  float vt_pix_clk;                   ///< Calculated based on PLL
  float op_pix_clk;                   ///< Calculated based on PLL
  uint16_t line_length;               ///< Calculated line length of blanking
  uint16_t frame_length;              ///< Calculated frame length of blanking

  float target_fps;                   ///< FPS wanted
  float real_fps;                     ///< Real calculated FPS
  float target_exposure;              ///< Target exposure time in ms
  float real_exposure;                ///< Real exposure time in ms

  float gain_green1;                  ///< Gain for the GreenR pixels [1., 63.50]
  float gain_blue;                    ///< Gain for the Blue pixels [1., 63.50]
  float gain_red;                     ///< Gain for the Red pixels [1., 63.50]
  float gain_green2;                  ///< Gain for the GreenB pixels [1., 63.50]

  uint16_t output_width;              ///< Output width
  uint16_t output_height;             ///< Output height
  uint16_t output_scaler;             ///< Output scaler
  uint16_t scaled_width;              ///< Width after corrected scaling
  uint16_t scaled_height;             ///< Height after corrected scaling
  uint16_t offset_x;                  ///< Offset from left in pixels
  uint16_t offset_y;                  ///< Offset from top in pixels

  uint16_t sensor_width;
  uint16_t sensor_height;

  uint8_t x_odd_inc;                  ///< X increment for subsampling (1,3,7,15,31 accepted)
  uint8_t y_odd_inc;                  ///< Y increment for subsampling (1,3,7,15,31 accepted)

  struct i2c_periph *i2c_periph;      ///< I2C peripheral used to communicate over
  struct i2c_transaction i2c_trans;   ///< I2C transaction for communication with CMOS chip

  float set_zoom;                      ///< Image zoom set point
  float set_offset_x;                  ///< Signed fractional offset from centre of image of original sensor [-0.5,0.5]
  float set_offset_y;                  ///< Signed fractional offset from centre of image of original sensor [-0.5,0.5]
};

/* ISP */
extern struct mt9f002_t mt9f002;

extern void mt9f002_init(struct mt9f002_t *mt);
extern void mt9f002_reset_exposure(struct mt9f002_t *mt);
extern void mt9f002_set_exposure(struct mt9f002_t *mt);
extern void mt9f002_reset_color(struct mt9f002_t *mt);
extern void mt9f002_set_gains(struct mt9f002_t *mt);

// settings to update resolution, color and exposure
float mt9f002_send_resolution;
float mt9f002_send_color;
float mt9f002_send_exposure;

// handlers for propagating settings
extern void mt9f002_setting_update_resolution(float in);
extern void mt9f002_setting_update_color(float in);
extern void mt9f002_setting_update_exposure(float in);

#endif /* MT9F002_H */
