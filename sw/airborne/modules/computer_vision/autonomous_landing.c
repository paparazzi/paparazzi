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

#define MEMORY 25

// Altitude control
float vz_ref = 0.25;
float vz_desired;

// Horizontal control
float velGain;
float vx_desired;
float vy_desired;

// Geofilter Settings
float marker_reached = 0.5;

// Counters
int waypoint_counter = 0;

// Positions of WP1
float wp_x[MEMORY];
float wp_y[MEMORY];

// Marker-detection timer
long previous_time;
float dt_sum = 0;
float dt = 0;
float dt_flight = 0;
float marker_lost = 2; // seconds

// Landing timer
float dt_sum_ld = 0;
float detect_ground = 0.5; // seconds

// Location of the centroid
uint32_t temp;

// Optical flow tracking
int of_tracking;

/*TODO: Why the altitude is reduced when changing modes from NAV to GUIDED? ASK!!!*/


struct image_t *autonomous_landing_func(struct image_t* img);
struct image_t *autonomous_landing_func(struct image_t* img)
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

  // Initialize the timer if the aircraft is not flying
  if (!autopilot_in_flight) {
    dt_flight = 0;
  }

  // Update the location of the centroid only if the marker is detected in the previous iteration
  if ((MARKER == 1) && (dt_flight > 3)) {
    temp = maxx;
    temp = temp << 16;
    temp += maxy;
    dt_sum = 0;
  }

  if (dt_sum <marker_lost) {

    // Change the flight mode from NAV to GUIDED
    if (AP_MODE_NAV == autopilot_mode) {
      autopilot_mode_auto2 = AP_MODE_GUIDED;
      autopilot_set_mode(AP_MODE_GUIDED);
    }

    // Process
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
    wp_x[MEMORY-1] = marker.x;
    wp_y[MEMORY-1] = marker.y;

    // Adaptive gain adjustment
    float term1, term2;
    if (marker.x <0) { term1 = -marker.x; } else { term1 = marker.x; }
    if (marker.y <0) { term2 = -marker.y; } else { term2 = marker.y; }
    velGain = (term1 + term2) / 4; /* TODO: Change the denominator */

    // Set velocities as offsets in NED frame
    float psi = stateGetNedToBodyEulers_f()->psi;
    vx_desired = cosf(-psi) * (marker.x * velGain) - sinf(-psi) * (marker.y * velGain);
    vy_desired = sinf(-psi) * (marker.x * velGain) + cosf(-psi) * (marker.y * velGain);

    if (!of_tracking) {
      // Follow the marker with velocity references
      guidance_h_set_guided_vel(vx_desired, vy_desired);
    } else {
      /* TODO: VELOCITY ESTIMATES MUST COME FROM OPTICFLOW NOT FROM OPTITRACK */
      guidance_h_set_guided_vel(0, 0);
    }

    // Absolute value
    if (wp_x[MEMORY-1] <0) { wp_x[MEMORY-1] = -wp_x[MEMORY-1]; }
    if (wp_y[MEMORY-1] <0) { wp_y[MEMORY-1] = -wp_y[MEMORY-1]; }

    // Check if the marker has been reached
    for (int i = 0; i <MEMORY; ++i) {
      if ((wp_x[i] < marker_reached) && (wp_y[i] < marker_reached)) {
        waypoint_counter++;
      }
    }

    /* TODO: Ground detection */
    // Landing
    if (waypoint_counter > (MEMORY - 1)) {

      // If the marker has been reached, start decreasing altitude
      if ((sonar_bebop.distance < 1) && (sonar_bebop.distance > 0.01)) {

        // Set vertical velocity
        vz_desired = 3 * vz_ref; /* TODO: Change this coefficient */

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
        vz_desired = vz_ref;
      }
    } else {

      // If the marker has not been reached, maintain altitude
      vz_desired = 0;
    }

    // Set a a reference for vertical velocity
    guidance_v_set_guided_vz(vz_desired);

    // Prepare variables for the next iteration
    waypoint_counter = 0;

    for (int i = 0; i <(MEMORY-1) ; ++i) {
      wp_x[i] = wp_x[i+1];
      wp_y[i] = wp_y[i+1];
    }

  } else { // Marker lost

    // Change the flight mode from GUIDED to NAV
    if (AP_MODE_GUIDED == autopilot_mode) {
      autopilot_mode_auto2 = AP_MODE_NAV;
      autopilot_set_mode(AP_MODE_NAV);
    }

    // Reinitialize the variables
    autonomous_landing_init_variables();
  }

  // Update variables
  previous_time = new_time;

  return NULL;
}


void autonomous_landing_init(void)
{
  // Platform tracking with OF deactivated
  of_tracking = 0;

  // Initialize georeference module
  georeference_init();

  // Initialize memory
  for (int i = 0; i <MEMORY ; ++i) {
    wp_x[i] = 10;
    wp_y[i] = 10;
  }

  // Desired velocities are set to zero
  vx_desired = 0;
  vy_desired = 0;
  vz_desired = 0;

  // Add detection function to CV
  cv_add_to_device(&MARKER_CAMERA, autonomous_landing_func);
}


uint8_t autonomous_landing_init_variables(void)
{
  // Reset landing timer to zero
  dt_sum_ld = 0;

  // Waypoint counter reset to zero
  waypoint_counter = 0;

  // Initialize memory
  for (int i = 0; i <MEMORY ; ++i) {
    wp_x[i] = 10;
    wp_y[i] = 10;
  }

  // Desired velocities are set to zero
  vx_desired = 0;
  vy_desired = 0;
  vz_desired = 0;

  return FALSE;
}