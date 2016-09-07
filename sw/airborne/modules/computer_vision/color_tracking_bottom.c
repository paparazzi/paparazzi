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
 * @file modules/computer_vision/color_tracking_bottom.c
 * @author IMAV 2016
 */

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
#include "modules/sonar/sonar_bebop.h"
#include <stdio.h>
#include <time.h>

// This parameter determines the number of iterations requires to start descending
#define MEMORY 25

// Run the module
bool RUN_MODULE_COLOR_BOTTOM;

// Choose color
int color = 0; // 0 = RED; 1 = BLUE

// Red color settings
uint8_t color_lum_min_red = 0;
uint8_t color_lum_max_red = 110;
uint8_t color_cb_min_red  = 52;
uint8_t color_cb_max_red  = 140;
uint8_t color_cr_min_red  = 140;
uint8_t color_cr_max_red  = 255;

/* TODO: Find color settings for blue objects */
// Blue color settings
uint8_t color_lum_min_blue = 70;
uint8_t color_lum_max_blue = 205;
uint8_t color_cb_min_blue  = 52;
uint8_t color_cb_max_blue  = 140;
uint8_t color_cr_min_blue  = 140;
uint8_t color_cr_max_blue  = 255;

// Reliable color detection
int blob_threshold_bottom = 50;

// Image-modification triggers
uint8_t modify_image_bottom = FALSE; // Image-modification trigger

// Target detection
struct results_color target_bottom;

// Marker detected in the bottom camera
uint8_t BOTTOM_MARKER;

// Altitude control
float vz_desired = 0.25;
float vz_bottom_ref;
float height_above_target = 0.6;

// Horizontal control
float vel_gain_color = 0.5; /* TODO: This requires more tuning and may cause instability with values higher than 0.7 */
float vx_bottom_ref;
float vy_bottom_ref;

// Geofilter Settings
float target_reached = 0.2;

// Counters
int centroid_counter;
float centroid_x[MEMORY];
float centroid_y[MEMORY];

// Location of the centroid
uint32_t temp;

// Marker-detection timer
long previous_time;
float dt_sum;
float dt;
float marker_lost; // seconds


struct image_t *color_tracking_bottom_func(struct image_t* img);
struct image_t *color_tracking_bottom_func(struct image_t* img)
{

  if (!RUN_MODULE_COLOR_BOTTOM) { return NULL; }

  // Compute new dt
  struct timespec spec;
  clock_gettime(CLOCK_REALTIME, &spec);
  long new_time = spec.tv_nsec / 1.0E6;
  long delta_t = new_time - previous_time;
  dt = ((float)delta_t) / 1000.0f;

  if (dt >0) {
    dt_sum += dt;
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  // COLORFILTER

  // Blob locator
  if (color == 0) {
    target_bottom = locate_blob(img,
                                color_lum_min_red, color_lum_max_red,
                                color_cb_min_red, color_cb_max_red,
                                color_cr_min_red, color_cr_max_red,
                                blob_threshold_bottom);
  } else if (color == 1) {
    target_bottom = locate_blob(img,
                                color_lum_min_blue, color_lum_max_blue,
                                color_cb_min_blue, color_cb_max_blue,
                                color_cr_min_blue, color_cr_max_blue,
                                blob_threshold_bottom);
  }

  // Display the marker location and center-lines.
  if ((modify_image_bottom) && (target_bottom.MARKER)) {
    int ti = target_bottom.maxy - 50;
    int bi = target_bottom.maxy + 50;
    struct point_t t = {target_bottom.maxx, ti}, b = {target_bottom.maxx, bi};

    int li = target_bottom.maxx - 50;
    int ri = target_bottom.maxx + 50;
    struct point_t l = {li, target_bottom.maxy}, r = {ri, target_bottom.maxy};

    image_draw_line(img, &t, &b);
    image_draw_line(img, &l, &r);
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  // NAVIGATION

  // Update the location of the centroid only if the marker is detected in the previous iteration
  if (target_bottom.MARKER) {
    temp = target_bottom.maxx;
    temp = temp << 16;
    temp += target_bottom.maxy;
    dt_sum = 0;
  }

  if ((dt_sum < marker_lost)) {

    // Bottom camera has a higher priority
    BOTTOM_MARKER = TRUE;

    // Do not change yaw
    guidance_h_set_guided_heading_rate(0);

    // Centroid
    uint16_t y = temp & 0x0000ffff;
    temp = temp >> 16;
    uint16_t x = temp & 0x0000ffff;

    struct camera_frame_t cam;
    cam.px = x/2;
    cam.py = y/2;
    cam.f = 400; // Focal length [px]
    cam.h = 240; // Frame height [px]
    cam.w = 320; // Frame width [px]

    // Update waypoint location and save its current (x,y) position
    struct centroid_t marker_location = georeference_project(&cam);

    // Update memory
    centroid_x[MEMORY-1] = marker_location.x;
    centroid_y[MEMORY-1] = marker_location.y;

    // Set velocities as offsets in NED frame
    vx_bottom_ref = vel_gain_color * marker_location.x;
    vy_bottom_ref = vel_gain_color * marker_location.y;

    // Saturation
    BoundAbs(vx_bottom_ref, 1);
    BoundAbs(vy_bottom_ref, 1);

    // Follow the marker with velocity references
    guidance_h_set_guided_vel(vx_bottom_ref, vy_bottom_ref);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    // HOVERING

    // Absolute value
    if (centroid_x[MEMORY-1] <0) { centroid_x[MEMORY-1] = -centroid_x[MEMORY-1]; }
    if (centroid_y[MEMORY-1] <0) { centroid_y[MEMORY-1] = -centroid_y[MEMORY-1]; }

    // Check if the marker has been reached
    for (int i = 0; i <MEMORY; ++i) {
      if ((centroid_x[i] < target_reached) && (centroid_y[i] < target_reached)) {
        centroid_counter++;
      }
    }

    /* TODO: Try hovering with the SONAR in the state of the drone.  */
    // Decrease altitude and hover above the marker
    if (centroid_counter > (MEMORY - 1)) {

      // If the marker has been reached, start decreasing altitude
      if (sonar_bebop.distance > height_above_target) {

        // Set vertical velocity
        vz_bottom_ref = vz_desired;
      } else {

        // Set vertical velocity
        vz_bottom_ref = 0;
      }
    } else {

      // If the marker has not been reached, maintain altitude
      vz_bottom_ref = 0;
    }

    // Set a a reference for vertical velocity
    guidance_v_set_guided_vz(vz_bottom_ref);

    // Prepare variables for the next iteration
    centroid_counter = 0;

    for (int i = 0; i <(MEMORY-1) ; ++i) {
      centroid_x[i] = centroid_x[i+1];
      centroid_y[i] = centroid_y[i+1];
    }

  } else {

    // Marker lost
    BOTTOM_MARKER = FALSE;

    // Reinitialize the variables
    init_variables();
  }

  // Update variables
  previous_time = new_time;

  return NULL;
}


void color_tracking_bottom_init(void)
{
  // Do not run the module automatically
  RUN_MODULE_COLOR_BOTTOM = FALSE;

  // Marker not detected in the bottom camera
  BOTTOM_MARKER = FALSE;

  // Initialize georeference module
  georeference_init();

  // Initialize memory
  for (int i = 0; i <MEMORY ; ++i) {
    centroid_x[i] = 10;
    centroid_y[i] = 10;
  }

  // Initialize variables
  dt_sum = 0;
  dt = 0;
  marker_lost = 2;
  centroid_counter = 0;

  // Desired velocities are set to zero
  vx_bottom_ref = 0;
  vy_bottom_ref = 0;
  vz_bottom_ref = 0;

  // Add detection function to CV
  cv_add_to_device(&COLOR_CAMERA_BOTTOM, color_tracking_bottom_func);
}


uint8_t init_variables(void)
{
  // Waypoint counter reset to zero
  centroid_counter = 0;

  // Initialize memory
  for (int i = 0; i <MEMORY ; ++i) {
    centroid_x[i] = 10;
    centroid_y[i] = 10;
  }

  // Desired velocities are set to zero
  vx_bottom_ref = 0;
  vy_bottom_ref = 0;
  vz_bottom_ref = 0;

  return FALSE;
}


uint8_t color_tracking_bottom_periodic(void) { return false; } /* currently no direct periodic functionality */


uint8_t start_color_tracking_bottom(void) { RUN_MODULE_COLOR_BOTTOM = TRUE; return false; }


uint8_t stop_color_tracking_bottom(void) { RUN_MODULE_COLOR_BOTTOM = FALSE; return false; }