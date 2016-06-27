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

#define MEMORY 25
#define PI 3.14159265359

// Choose color
int color = 0; // 0 = RED; 1 = BLUE

// Red color settings
uint8_t color_lum_min_red = 70;
uint8_t color_lum_max_red = 205;
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
int blob_threshold_bottom = 1000;

// Image-modification triggers
uint8_t modify_image_bottom = FALSE; // Image-modification trigger

// Target detection
struct results_color destination;

// Marker detected in the bottom camera
uint8_t BOTTOM_MARKER = FALSE;

// Altitude control
float vz_bottom_ref = 0.25;
float height_above_target = 1;

// Horizontal control
float velGain_bottom;
float vx_bottom_ref;
float vy_bottom_ref;

// Geofilter Settings
float target_reached = 0.5;

// Counters
int centroid_counter = 0;
float centroid_x[MEMORY];
float centroid_y[MEMORY];

// Location of the centroid
uint32_t temp;

// Marker-detection timer
long previous_time;
float dt_sum = 0;
float dt = 0;
float dt_flight = 0;
float marker_lost = 2; // seconds


struct image_t *color_tracking_bottom_func(struct image_t* img);
struct image_t *color_tracking_bottom_func(struct image_t* img)
{

  // Compute new dt
  struct timespec spec;
  clock_gettime(CLOCK_REALTIME, &spec);
  long new_time = spec.tv_nsec / 1.0E6;
  long delta_t = new_time - previous_time;
  dt = ((float)delta_t) / 1000.0f;

  if (dt >0) {
    dt_sum += dt;
    dt_flight += dt;
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  // COLORFILTER

  // Blob locator
  if (color == 0) {
    destination = locate_blob(img,
                              color_lum_min_red, color_lum_max_red,
                              color_cb_min_red, color_cb_max_red,
                              color_cr_min_red, color_cr_max_red,
                              blob_threshold_bottom);
  } else if (color == 1) {
    destination = locate_blob(img,
                              color_lum_min_blue, color_lum_max_blue,
                              color_cb_min_blue, color_cb_max_blue,
                              color_cr_min_blue, color_cr_max_blue,
                              blob_threshold_bottom);
  }


  // Display the marker location and center-lines.
  if ((modify_image_bottom) && (destination.MARKER)) {
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
  // NAVIGATION

//  // Initialize the timer if the aircraft is not flying
//  if (!autopilot_in_flight) {
//    dt_flight = 0;
//  }
//
//  // Update the location of the centroid only if the marker is detected in the previous iteration
//  if (destination.MARKER && (dt_flight > 3)) {
//    temp = destination.maxx;
//    temp = temp << 16;
//    temp += destination.maxy;
//    dt_sum = 0;
//  }
//
//  if (dt_sum <marker_lost) {
//    BOTTOM_MARKER = TRUE;
//
//    // Change the flight mode from NAV to GUIDED
//    if (AP_MODE_NAV == autopilot_mode) {
//      autopilot_mode_auto2 = AP_MODE_GUIDED;
//      autopilot_set_mode(AP_MODE_GUIDED);
//    }
//
//    // Process
//    uint16_t y = temp & 0x0000ffff;
//    temp = temp >> 16;
//    uint16_t x = temp & 0x0000ffff;
//
//    struct camera_frame_t cam;
//    cam.px = x/2;
//    cam.py = y/2;
//    cam.f = 400; // Focal length [px]
//    cam.h = 240; // Frame height [px]
//    cam.w = 320; // Frame width [px]
//
//    // Update waypoint location and save its current (x,y) position
//    struct centroid_t marker_location = georeference_project(&cam);
//
//    // Update memory
//    centroid_x[MEMORY-1] = marker_location.x;
//    centroid_y[MEMORY-1] = marker_location.y;
//
//    // Adaptive gain adjustment
//    float term1, term2;
//    if (marker_location.x <0) { term1 = -marker_location.x; } else { term1 = marker_location.x; }
//    if (marker_location.y <0) { term2 = -marker_location.y; } else { term2 = marker_location.y; }
//    velGain_bottom = (term1 + term2) / 4; /* TODO: Change the denominator */
//
//    // Set velocities as offsets in NED frame
//    float psi = stateGetNedToBodyEulers_f()->psi;
//    vx_bottom_ref = cosf(-psi) * (marker_location.x * velGain_bottom) - sinf(-psi) * (marker_location.y * velGain_bottom);
//    vy_bottom_ref = sinf(-psi) * (marker_location.x * velGain_bottom) + cosf(-psi) * (marker_location.y * velGain_bottom);
//
//    // Follow the marker with velocity references
//    guidance_h_set_guided_vel(vx_bottom_ref, vy_bottom_ref);
//
//    // Absolute value
//    if (centroid_x[MEMORY-1] <0) { centroid_x[MEMORY-1] = -centroid_x[MEMORY-1]; }
//    if (centroid_y[MEMORY-1] <0) { centroid_y[MEMORY-1] = -centroid_y[MEMORY-1]; }
//
//    // Check if the marker has been reached
//    for (int i = 0; i <MEMORY; ++i) {
//      if ((centroid_x[i] < target_reached) && (centroid_y[i] < target_reached)) {
//        centroid_counter++;
//      }
//    }
//
//    // Landing
//    if (centroid_counter > (MEMORY - 1)) {
//
//      // If the marker has been reached, start decreasing altitude
//      if (sonar_bebop.distance > height_above_target) {
//
//        // Set vertical velocity
//        guidance_v_set_guided_vz(vz_bottom_ref);
//      }
//    } else {
//
//      // If the marker has not been reached, maintain altitude
//      guidance_v_set_guided_vz(0);
//    }
//
//    // Prepare variables for the next iteration
//    centroid_counter = 0;
//
//    for (int i = 0; i <(MEMORY-1) ; ++i) {
//      centroid_x[i] = centroid_x[i+1];
//      centroid_y[i] = centroid_y[i+1];
//    }
//
//
//  } else { // Marker lost
//
//    BOTTOM_MARKER = FALSE;
//
//    // Reinitialize the variables
//    init_variables();
//  }

  // Update variables
  previous_time = new_time;

  return NULL;
}


void color_tracking_bottom_init(void)
{
  // Initialize georeference module
  georeference_init();

  // Initialize memory
  for (int i = 0; i <MEMORY ; ++i) {
    centroid_x[i] = 10;
    centroid_y[i] = 10;
  }

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