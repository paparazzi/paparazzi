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
#include "modules/computer_vision/color_tracking_bottom.h"
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "modules/computer_vision/imav2016markers.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "subsystems/datalink/telemetry.h"
#include "generated/flight_plan.h"
#include "autopilot.h"
#include <stdio.h>
#include <time.h>

// Run the module
bool RUN_MODULE_COLOR_FRONT;

// Reliable color detection
int blob_threshold_front = 0;

// Image-modification triggers
uint8_t modify_image_front = FALSE; // Image-modification trigger

// Target detection
struct results_color target_front;

// Navigation: forward velocity
float vel_ref = 0.75;
float yaw_rate_front_ref = 1.5; /* TODO: This requires more tuning. */


struct image_t *color_tracking_front_func(struct image_t* img);
struct image_t *color_tracking_front_func(struct image_t* img)
{

  if (!RUN_MODULE_COLOR_FRONT) { return NULL; }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  // COLORFILTER

  // Blob locator
  if (color == 0) {
    target_front = locate_blob(img,
                               color_lum_min_red, color_lum_max_red,
                               color_cb_min_red, color_cb_max_red,
                               color_cr_min_red, color_cr_max_red,
                               blob_threshold_front);
  } else if (color == 1) {
    target_front = locate_blob(img,
                               color_lum_min_blue, color_lum_max_blue,
                               color_cb_min_blue, color_cb_max_blue,
                               color_cr_min_blue, color_cr_max_blue,
                               blob_threshold_front);
  }


  // Display the marker location and center-lines.
  if ((modify_image_front) && (target_front.MARKER)) {
    int ti = target_front.maxy - 50;
    int bi = target_front.maxy + 50;
    struct point_t t = {target_front.maxx, ti}, b = {target_front.maxx, bi};

    int li = target_front.maxx - 50;
    int ri = target_front.maxx + 50;
    struct point_t l = {li, target_front.maxy}, r = {ri, target_front.maxy};

    image_draw_line(img, &t, &b);
    image_draw_line(img, &l, &r);
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  // NAVIGATION

  if ((target_front.MARKER) && (!BOTTOM_MARKER)) {

    // Maintain altitude
    guidance_v_set_guided_vz(0);

    // Compute the location of the centroid
    int centroid_x = target_front.maxx;

    // Compute the location of the centroid wrt the center of the frame
    int centroid_x_centered = centroid_x - (img->w)/2;

    // Set yaw rate based on the location of the color
    float yaw_rate = yaw_rate_front_ref * centroid_x_centered * 2. / (float)img->w;
    guidance_h_set_guided_heading_rate(yaw_rate);

    // Detect if the marker is in the middle of the frame
    if ((centroid_x_centered > -5) && (centroid_x_centered < 5)) {

      // Set velocities as offsets in NED frame
      guidance_h_set_guided_body_vel(vel_ref, 0.);

    } else {

      // Hold position
      guidance_h_set_guided_body_vel(0, 0);
    }

  } else if ((!target_front.MARKER) && (!BOTTOM_MARKER)) {

    // Change yaw until red is detected
    guidance_h_set_guided_heading_rate(yaw_rate_front_ref / 3.);

    // Maintain altitude
    guidance_v_set_guided_vz(0);

  }
  return NULL;
}


void color_tracking_front_init(void)
{
  // Do not run the module automatically
  RUN_MODULE_COLOR_FRONT = FALSE;

  // Add detection function to CV
  cv_add_to_device(&COLOR_CAMERA_FRONT, color_tracking_front_func);
}


uint8_t color_tracking_front_periodic(void) { return false; } /* currently no direct periodic functionality */


uint8_t start_color_tracking_front(void) { RUN_MODULE_COLOR_FRONT = TRUE; return false; }


uint8_t stop_color_tracking_front(void) { RUN_MODULE_COLOR_FRONT = FALSE; return false; }