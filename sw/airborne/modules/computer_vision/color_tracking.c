/*
 * Copyright (C) 2016 - IMAV 2016
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
 * @file modules/computer_vision/marker_tracking.c
 * @author IMAV 2016
 */

#include "modules/computer_vision/color_tracking.h"
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "modules/computer_vision/imav2016markers.h"
#include "subsystems/datalink/telemetry.h"
#include <stdio.h>
#include <time.h>

// Red color settings
uint8_t color_lum_min = 70; // 105
uint8_t color_lum_max = 205;
uint8_t color_cb_min  = 52;
uint8_t color_cb_max  = 140;
uint8_t color_cr_min  = 140;
uint8_t color_cr_max  = 255;
uint8_t modify_image  = FALSE; // Image-modification trigger

// Reliable color detection
int color_detection = 5000;

// Output
uint8_t MARKER  = FALSE;
int maxx;
int maxy;


struct image_t *color_tracking_func(struct image_t* img);
struct image_t *color_tracking_func(struct image_t* img)
{
  // Initialize the triggering
  MARKER  = FALSE;

  // Colorfilter
  struct colorfilter_t color_info = colorfilter(img, img, color_lum_min,color_lum_max, color_cb_min,color_cb_max,
                                                color_cr_min, color_cr_max, modify_image);

  if (color_info.cnt > color_detection) {
    // Compute the location of the centroid
    maxx = color_info.x_pos / color_info.cnt;
    maxy = color_info.y_pos / color_info.cnt;

    // Marker detected
    MARKER = TRUE;
  }

  // Display the marker location and center-lines.
  if (modify_image) {

    int ti = MARKER ? maxy - 50 : 0;
    int bi = MARKER ? maxy + 50 : img->h;
    struct point_t t = {maxx, ti}, b = {maxx, bi};

    int li = MARKER ? maxx - 50 : 0;
    int ri = MARKER ? maxx : img->w;
    struct point_t l = {li, maxy}, r = {ri, maxy};

    image_draw_line(img, &t, &b);
    image_draw_line(img, &l, &r);
  }

  return NULL;
}


void color_tracking_init(void)
{
  // Initialize the triggering
  MARKER  = FALSE;

  // Add detection function to CV
  cv_add_to_device(&COLOR_CAMERA, color_tracking_func);
}