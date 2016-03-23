/*
 * Copyright (C) 2015
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
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/colorfilter.c
 */

// Own header
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/colorfilter.h"

#include <stdio.h>

// Filter Settings
uint8_t color_lum_min = 105;
uint8_t color_lum_max = 205;
uint8_t color_cb_min  = 52;
uint8_t color_cb_max  = 140;
uint8_t color_cr_min  = 180;
uint8_t color_cr_max  = 255;

// Result
int color_count = 0;

// Function
struct image_t* colorfilter_func(struct image_t* img);
struct image_t* colorfilter_func(struct image_t* img)
{
  // Filter
  color_count = image_yuv422_colorfilt(img,img,
      color_lum_min,color_lum_max,
      color_cb_min,color_cb_max,
      color_cr_min,color_cr_max
      );

  return img; // Colorfilter did not make a new image
}

void colorfilter_init(void)
{
  cv_add(colorfilter_func);
}

