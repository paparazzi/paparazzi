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

/* define required ouput image size */
#ifndef MT9F002_OUTPUT_HEIGHT
#define MT9F002_OUTPUT_HEIGHT 822  // full resolution 3288
#endif

#ifndef MT9F002_OUTPUT_WIDTH
#define MT9F002_OUTPUT_WIDTH 1152 // full resolution 4608
#endif

#ifndef MT9F002_INITIAL_OFFSET_X
#define MT9F002_INITIAL_OFFSET_X 0. // signed fractional offset from centre of image of original sensor [-0.5,0.5]
#endif

#ifndef MT9F002_INITIAL_OFFSET_Y
#define MT9F002_INITIAL_OFFSET_Y 0. // signed fractional offset from centre of image of original sensor [-0.5,0.5]
#endif

/** Our output is only OUTPUT_SCALER of the pixels we take of the sensor
 * It is programmable in 1/16 steps determined by ScaleFactor = 16/scale_m.
 * Legal values for scale_m are 16 through 128, giving you the ability to scale from
 * 1:1 to 1:8 (with m=128).
 *  Example:
 *  output_width = 512
 *  output_height = 830
 *  output_scaler = 0.25
 *  We now get an image of 512 by 830 which contains a "compressed version"
 *  of what would normally be an image of 2048 by 3320 ISP (4608H x 2592V sensor)
 */
#ifndef MT9F002_OUTPUT_SCALER
#define MT9F002_OUTPUT_SCALER 1.
#endif

/** Exposure of the front camera of the bebop. Experimental values:
 * Outside: 15
 * Inside well lit: 30
 * Inside poorly lit: 60
 */
#ifndef MT9F002_TARGET_EXPOSURE
#define MT9F002_TARGET_EXPOSURE 30
#endif

#ifndef MT9F002_TARGET_FPS
#define MT9F002_TARGET_FPS 15
#endif

/* Set the colour balance gains */
#ifndef MT9F002_GAIN_GREEN1
#define MT9F002_GAIN_GREEN1 3.0
#endif

#ifndef MT9F002_GAIN_GREEN2
#define MT9F002_GAIN_GREEN2 3.0
#endif

#ifndef MT9F002_GAIN_RED
#define MT9F002_GAIN_RED 3.0
#endif

#ifndef MT9F002_GAIN_BLUE
#define MT9F002_GAIN_BLUE 4.0
#endif

/* Set pixel increment value to implement subsampling */
/* Supported values for MT9F002_X_ODD_INC_VAL are 3, 7, 15 and 31 */
#ifndef MT9F002_X_ODD_INC_VAL
#define MT9F002_X_ODD_INC_VAL 1
#endif

/* Supported values for MT9F002_Y_ODD_INC_VAL are 1, 3 and 7 */
#ifndef MT9F002_Y_ODD_INC_VAL
#define MT9F002_Y_ODD_INC_VAL 1
#endif

// parameters for undistortion
#ifndef MT9F002_FOCAL_X
#define MT9F002_FOCAL_X 311.59304538f
#endif
#ifndef MT9F002_FOCAL_Y
#define MT9F002_FOCAL_Y 313.01338397f
#endif
#ifndef MT9F002_CENTER_X
#define MT9F002_CENTER_X 158.37457814f
#endif
#ifndef MT9F002_CENTER_Y
#define MT9F002_CENTER_Y 326.49375925f
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

  float gain_green1;                  ///< Gain for the GreenR pixels [1.5 -> 63.50]
  float gain_blue;                    ///< Gain for the Blue pixels [1.5 -> 63.50]
  float gain_red;                     ///< Gain for the Red pixels [1.5 -> 63.50]
  float gain_green2;                  ///< Gain for the GreenB pixels [1.5 -> 63.50]

  uint16_t output_width;              ///< Output width
  uint16_t output_height;             ///< Output height
  float output_scaler;                ///< Output scale
  uint16_t scaled_width;              ///< Width after corrected scaling
  uint16_t scaled_height;             ///< Height after corrected scaling
  float offset_x;                  ///< Offset from left in pixels
  float offset_y;                  ///< Offset from top in pixels

  uint8_t x_odd_inc;                  ///< X increment for subsampling (1,3,7,15,31 accepted)
  uint8_t y_odd_inc;                  ///< Y increment for subsampling (1,3,7 accepted)

  struct i2c_periph *i2c_periph;      ///< I2C peripheral used to communicate over
  struct i2c_transaction i2c_trans;   ///< I2C transaction for comminication with CMOS chip
};

void mt9f002_init(struct mt9f002_t *mt);
void mt9f002_set_resolution(struct mt9f002_t *mt);
void mt9f002_set_exposure(struct mt9f002_t *mt);
void mt9f002_set_gains(struct mt9f002_t *mt);

#endif /* MT9F002_H */
