/*
 * $Id$
 *  
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA. 
 *
 */
/** \file cam.c
 *  \brief Regroup functions to control the camera
 *
 */

#include <math.h>
#include "cam.h"
#include "nav.h"
#include "autopilot.h"
#include "flight_plan.h"
#include "estimator.h"
#include "link_fbw.h"

#define MIN_PPRZ_CAM ((int16_t)(MAX_PPRZ * 0.05))
#define DELTA_ALPHA 0.2

#define MAX_CAM_ROLL M_PI/2
#define MAX_CAM_PITCH M_PI/3

#define target_alt GROUND_ALT

float phi_c, theta_c;
float target_x, target_y;

void cam_manual( void ) {
  if (pprz_mode == PPRZ_MODE_AUTO2) {
    static pprz_t cam_roll, cam_pitch;
    int16_t yaw = from_fbw.channels[RADIO_YAW];
    if (yaw > MIN_PPRZ_CAM || yaw < -MIN_PPRZ_CAM) { 
      cam_roll += FLOAT_OF_PPRZ(yaw, 0, 300.);
      cam_roll = TRIM_PPRZ(cam_roll);
    }
    int16_t pitch = from_fbw.channels[RADIO_PITCH];
    if (pitch > MIN_PPRZ_CAM || pitch < -MIN_PPRZ_CAM) {
      cam_pitch += FLOAT_OF_PPRZ(pitch, 0, 300.);
      cam_pitch = TRIM_PPRZ(cam_pitch);
    }
    to_fbw.channels[RADIO_GAIN1] = cam_roll;
    to_fbw.channels[RADIO_CALIB] = cam_pitch;
  }
}

void cam_nadir( void ) {
  phi_c = -estimator_phi;
  theta_c = -estimator_theta;
}

void cam_target( void ) {
  float h = estimator_z - target_alt;
  float c_psi = cos(estimator_psi);
  float s_psi = sin(estimator_psi);
  phi_c = atan((target_x*c_psi - target_y*s_psi - estimator_x) / h) - estimator_phi;
  theta_c = atan((target_x*s_psi + target_y*c_psi - estimator_y) / h)- estimator_theta;
}

#define MAX_DIST_TARGET 500.

void cam_manual_target( void ) {
  int16_t yaw = from_fbw.channels[RADIO_YAW];
  if (yaw > MIN_PPRZ_CAM || yaw < -MIN_PPRZ_CAM) {
    target_x += FLOAT_OF_PPRZ(yaw, 0, -20.);
    target_x = Min(target_x, MAX_DIST_TARGET + estimator_x);
    target_x = Max(target_x, -MAX_DIST_TARGET + estimator_x);
  }
  int16_t pitch = from_fbw.channels[RADIO_PITCH];
  if (pitch > MIN_PPRZ_CAM || pitch < -MIN_PPRZ_CAM) {
    target_y += FLOAT_OF_PPRZ(pitch, 0, -20.);
    target_y = Min(target_y, MAX_DIST_TARGET + estimator_y);
    target_y = Max(target_y, -MAX_DIST_TARGET + estimator_y);
  }
  cam_target();
}

void cam_waypoint_target( uint8_t wp ) {
  target_x = waypoints[wp].x;
  target_y = waypoints[wp].y;
  cam_target();
}

void cam_carrot( void ) {
  target_x = carrot_x;
  target_y = carrot_y;
  cam_target();
}
