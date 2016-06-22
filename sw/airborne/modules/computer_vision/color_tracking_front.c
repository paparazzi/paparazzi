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
 * @file modules/computer_vision/color_tracking_front.c
 * @author IMAV 2016
 */

#include "modules/computer_vision/color_tracking_front.h"
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "modules/computer_vision/imav2016markers.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "subsystems/datalink/telemetry.h"
#include "generated/flight_plan.h"
#include "autopilot.h"
#include <stdio.h>
#include <time.h>

#define PI 3.14159265359

// Red color settings
uint8_t color_lum_min_front = 70;
uint8_t color_lum_max_front = 205;
uint8_t color_cb_min_front  = 52;
uint8_t color_cb_max_front  = 140;
uint8_t color_cr_min_front  = 140;
uint8_t color_cr_max_front  = 255;
int blob_threshold_front = 1000; // Reliable red detection

// Image-modification triggers
uint8_t modify_image_front  = FALSE; // Image-modification trigger

// Helipad detection
struct results_color destination;


struct image_t *color_tracking_front_func(struct image_t* img);
struct image_t *color_tracking_front_func(struct image_t* img)
{

  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  // COLORFILTER

  // Blob locator
  destination = locate_blob(img,
                             color_lum_min_front, color_lum_max_front,
                             color_cb_min_front, color_cb_max_front,
                             color_cr_min_front, color_cr_max_front,
                             blob_threshold_front);

  // Display the marker location and center-lines.
  if ((modify_image_front) && (destination.MARKER)) {
    int ti = destination.maxy - 50;
    int bi = destination.maxy + 50;
    struct point_t t = {destination.maxx, ti}, b = {destination.maxx, bi};

    int li = destination.maxx - 50;
    int ri = destination.maxx + 50;
    struct point_t l = {li, destination.maxy}, r = {ri, destination.maxy};

    image_draw_line(img, &t, &b);
    image_draw_line(img, &l, &r);
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////

  if (destination.MARKER) {
    // Compute the location of the centroid
    int centroid_x = destination.maxx;

    // Compute the location of the centroid wrt the center of the frame
    int centroid_x_centered = centroid_x - (img->w)/2;

    // Change the flight mode from NAV to GUIDED
    if (AP_MODE_NAV == autopilot_mode) {
      autopilot_mode_auto2 = AP_MODE_GUIDED;
      autopilot_set_mode(AP_MODE_GUIDED);
    }

    // Hold position
    guidance_h_set_guided_vel(0, 0);

    // Set yaw rate based on the location of the color
    float yaw_rate = 0;
    if (centroid_x_centered > 0) {yaw_rate = 0.15;} else if (centroid_x_centered < 0) {yaw_rate = -0.15;}
    guidance_h_set_guided_heading_rate(yaw_rate);

  } else {

    // Change the flight mode from GUIDED to NAV
    if (AP_MODE_GUIDED == autopilot_mode) {
      autopilot_mode_auto2 = AP_MODE_NAV;
      autopilot_set_mode(AP_MODE_NAV);
    }

  }

  return NULL;
}


void color_tracking_front_init(void)
{
  // Add detection function to CV
  cv_add_to_device(&COLOR_CAMERA_FRONT, color_tracking_front_func);
}