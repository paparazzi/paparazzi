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
#include "modules/computer_vision/grass_detector.h"
#include <stdio.h>
#include <stdbool.h>

#include "modules/computer_vision/lib/vision/image.h"

#ifndef GRASS_DETECTOR_CAMERA
#define GRASS_DETECTOR_CAMERA bottom_camera
#endif
#ifndef GRASS_DETECTOR_FPS
#define GRASS_DETECTOR_FPS 0 ///< Default FPS (zero means run at camera fps)
#endif

struct video_listener *listener = NULL;

// Filter Settings
uint8_t color_lum_min = 0;
uint8_t color_lum_max = 255;
uint8_t color_cb_min  = 0;
uint8_t color_cb_max  = 110;
uint8_t color_cr_min  = 0;
uint8_t color_cr_max  = 115;

grass_detector cv_grass_detector;

double range_threshold          = 0.09;
double settings_count_threshold = 0.12;

// Result
uint32_t grass_count = 0;
int32_t x_c = 0;
int32_t y_c = 0;

// Function
struct image_t *grass_detector_func(struct image_t *img);
static uint32_t find_grass_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc);

struct image_t *grass_detector_func(struct image_t *img)
{
  uint32_t count_threshold =
      (uint32_t) round(settings_count_threshold * img->w * img->h);
  // Filter and find centroid
  grass_count = find_grass_centroid(img, &x_c, &y_c);
  if (grass_count < count_threshold) {
    cv_grass_detector.inside = GRASS_UNSURE;
  } else {
    cv_grass_detector.range = hypot(x_c, y_c) / hypot(img->w * 0.5, img->h * 0.5);
    if (cv_grass_detector.range < range_threshold) {
      cv_grass_detector.inside = GRASS_INSIDE;
    } else {
      cv_grass_detector.inside = GRASS_OUTSIDE;
      cv_grass_detector.angle = atan2(x_c, y_c);  // x=y rotate 90deg so 0 points forward
      //printf("centroid: (%d, %d) r: %4.2f a: %4.2f\n", x_c, y_c, cv_grass_detector.range, cv_grass_detector.angle / M_PI * 180);
    }
  }
  return NULL;
}

void grass_detector_init(void)
{
  cv_grass_detector.inside = GRASS_UNSURE;
  cv_grass_detector.angle = 0.0;
  cv_grass_detector.range = 0.0;
  listener = cv_add_to_device(&GRASS_DETECTOR_CAMERA, grass_detector_func, GRASS_DETECTOR_FPS);
}

uint32_t find_grass_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc)
{
  uint32_t cnt = 0;
  uint32_t tot_x = 0;
  uint32_t tot_y = 0;
  uint8_t *buffer = img->buf;

  // Go trough all the pixels
  for (uint16_t y = 0; y < img->h; y++) {
    for (uint16_t x = 0; x < img->w; x ++) {
      // Check if the color is inside the specified values
      uint8_t *yp, *up, *vp;
      if (x % 2 == 0) {
        // Even x
        up = &buffer[y * 2 * img->w + 2 * x];     // U
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
        vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
        //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
      } else {
        // Even x
        up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
        //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
        vp = &buffer[y * 2 * img->w + 2 * x];     // V
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
      }
      if ( (*yp >= color_lum_min)
        && (*yp <= color_lum_max)
        && (*up >= color_cb_min)
        && (*up <= color_cb_max)
        && (*vp >= color_cr_min)
        && (*vp <= color_cr_max)) {
        cnt ++;
        tot_x += x;
        tot_y += y;
        *yp = 255;
      }
    }
  }
  if (cnt > 0) {
    *p_xc = (int32_t) round(tot_x / ((double) cnt) - img->w / 2.0);
    *p_yc = (int32_t) round(img->h / 2.0 - tot_y / ((double) cnt));
  } else {
    *p_xc = 0;
    *p_yc = 0;
  }
  return cnt;
}
