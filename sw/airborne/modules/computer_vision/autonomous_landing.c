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
 * @file modules/computer_vision/autonomous_landing.c
 * @author IMAV 2016
 */

#include "modules/computer_vision/autonomous_landing.h"
#include "modules/computer_vision/cv_helipad.h"
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
bool RUN_MODULE_LANDING;

// Altitude control
float vz_ref = 0.25;
float vz_bottom_ref;

// Horizontal control
float vel_gain_landing = 0.5; /* TODO: This requires more tuning based on the amount of fps. */
float vx_bottom_ref;
float vy_bottom_ref;

// Geofilter Settings
float marker_reached = 0.2;

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

// Landing timer
float dt_sum_ld = 0;
float detect_ground = 0.5; // seconds


struct image_t *autonomous_landing_func(struct image_t* img);
struct image_t *autonomous_landing_func(struct image_t* img)
{
  if (!RUN_MODULE_LANDING) { return NULL; }

  // Compute new dt
  struct timespec spec;
  clock_gettime(CLOCK_REALTIME, &spec);
  long new_time = spec.tv_nsec / 1.0E6;
  long delta_t = new_time - previous_time;
  dt = ((float)delta_t) / 1000.0f;

  if (dt >0) { dt_sum += dt; }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  // NAVIGATION

  // Update the location of the centroid only if the marker is detected in the previous iteration
  pthread_mutex_lock(&marker_mutex);
  if (MARKER) {
    temp = maxx;
    temp = temp << 16;
    temp += maxy;
    dt_sum = 0;
  }
  pthread_mutex_unlock(&marker_mutex);

  if (dt_sum <marker_lost) {

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
    struct centroid_t marker = georeference_project(&cam);

    // Update memory
    centroid_x[MEMORY-1] = marker.x;
    centroid_y[MEMORY-1] = marker.y;

    // Set velocities as offsets in NED frame
    vx_bottom_ref = vel_gain_landing * marker.x;
    vy_bottom_ref = vel_gain_landing * marker.y;

    // Saturation
    BoundAbs(vx_bottom_ref, 1);
    BoundAbs(vy_bottom_ref, 1);

    // Follow the marker with velocity references
    guidance_h_set_guided_vel(vx_bottom_ref, vy_bottom_ref);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    // LANDING

    // Absolute value
    if (centroid_x[MEMORY-1] <0) { centroid_x[MEMORY-1] = -centroid_x[MEMORY-1]; }
    if (centroid_y[MEMORY-1] <0) { centroid_y[MEMORY-1] = -centroid_y[MEMORY-1]; }

    // Check if the marker has been reached
    for (int i = 0; i <MEMORY; ++i) {
      if ((centroid_x[i] < marker_reached) && (centroid_y[i] < marker_reached)) {
        centroid_counter++;
      }
    }

    /* TODO: Improve ground detection */
    // Landing
    if (centroid_counter > (MEMORY - 1)) {

      // If the marker has been reached, start decreasing altitude
      if ((sonar_bebop.distance < 1) && (sonar_bebop.distance > 0.01)) {

        // Set a higher vertical velocity
        vz_bottom_ref = 3 * vz_ref; /* TODO: This requires more tuning. */

        // Increase the landing timer
        if (dt >0) {
          dt_sum_ld +=dt;
        }

        // Detect ground and kill the engines
        if ((dt_sum_ld > detect_ground) && (stateGetSpeedEnu_f()->z > 0)) {
          if (AP_MODE_GUIDED == autopilot_mode) {
            autopilot_set_motors_on(FALSE);
          }
        }
      } else {

        // If not, descend with a lower vertical speed
        vz_bottom_ref = vz_ref;
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

    /* TODO: What should we do when the landing marker is lost?  */
    // Hold position
    guidance_h_set_guided_body_vel(0, 0);

    // Reinitialize the variables
    autonomous_landing_init_variables();
  }

  // Update variables
  previous_time = new_time;

  return NULL;
}


void autonomous_landing_init(void)
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

  // Initialize variables
  dt_sum = 0;
  dt = 0;
  marker_lost = 2;
  centroid_counter = 0;

  // TODO: NOT NEEDED ANYMORE! call function directly
  cv_add_to_device(&MARKER_CAMERA, autonomous_landing_func);
}


uint8_t autonomous_landing_init_variables(void)
{
  // Do not run the module automatically
  RUN_MODULE_LANDING = FALSE;

  // Reset landing timer to zero
  dt_sum_ld = 0;

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

uint8_t autonomous_landing_periodic(void) { return false; } /* currently no direct periodic functionality */


uint8_t start_autonomous_landing(void) { RUN_MODULE_LANDING = TRUE; return false; }


uint8_t stop_autonomous_landing(void) { RUN_MODULE_LANDING = FALSE; return false; }