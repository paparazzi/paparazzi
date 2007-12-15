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
 *  \brief Pan/Tilt camera library
 *
 */

#include <math.h>
#include "cam.h"
#include "nav.h"
#include "autopilot.h"
#include "flight_plan.h"
#include "estimator.h"
#include "inter_mcu.h"
#include "traffic_info.h"
#ifdef POINT_CAM
#include "point.h"
#endif // POINT_CAM

#ifdef CAM_PITCH_NEUTRAL
#if (CAM_PITCH_MAX == CAM_PITCH_NEUTRAL)
#error CAM_PITCH_MAX has to be different from CAM_PITCH_NEUTRAL
#endif
#if (CAM_PITCH_NEUTRAL == CAM_PITCH_MIN)
#error CAM_PITCH_MIN has to be different from CAM_PITCH_NEUTRAL
#endif
#endif

#define MIN_PPRZ_CAM ((int16_t)(MAX_PPRZ * 0.05))
#define DELTA_ALPHA 0.2

#define MAX_CAM_ROLL M_PI/2
#define MAX_CAM_PITCH M_PI/3

#ifdef CAM_PHI0
float phi_c = RadOfDeg(CAM_PHI0);
#else
float phi_c;
#endif

#ifdef CAM_THETA0
float theta_c = RadOfDeg(CAM_THETA0);
#else
float theta_c;
#endif

float target_x, target_y, target_alt;

int use_hatch_cam = 0;
int cam_mode_auto = 1;

#ifdef MOBILE_CAM

void cam_manual( void ) {
  if (pprz_mode == PPRZ_MODE_AUTO2) {
    static pprz_t cam_roll, cam_pitch;
    int16_t yaw = fbw_state->channels[RADIO_YAW];
    if (yaw > MIN_PPRZ_CAM || yaw < -MIN_PPRZ_CAM) { 
      cam_roll += FLOAT_OF_PPRZ(yaw, 0, 300.);
      cam_roll = TRIM_PPRZ(cam_roll);
    }
    int16_t pitch = fbw_state->channels[RADIO_PITCH];
    if (pitch > MIN_PPRZ_CAM || pitch < -MIN_PPRZ_CAM) {
      cam_pitch += FLOAT_OF_PPRZ(pitch, 0, 300.);
      cam_pitch = TRIM_PPRZ(cam_pitch);
    }
    ap_state->commands[COMMAND_CAM_ROLL] = cam_roll;
    ap_state->commands[COMMAND_CAM_PITCH] = cam_pitch;
  }
}

void cam_nadir( void ) {
  phi_c = -estimator_phi;
  theta_c = -estimator_theta;
}

void cam_target( void ) {

#ifdef POINT_CAM
  /* phi   fRoll */
  /* theta fTilt */

  float cam_pitch;

  if (cam_mode_auto != 0)
  {
    vPoint(estimator_x, estimator_y, estimator_z,
           estimator_hspeed_dir, estimator_theta, estimator_phi,
           target_x, target_y, target_alt,
           &phi_c, &theta_c);
  }

  if (theta_c < RadOfDeg(CAM_PITCH_MIN - 30)) theta_c = RadOfDeg(CAM_PITCH_MAX);
  
  if (theta_c > RadOfDeg(CAM_PITCH_NEUTRAL))
  {
//    cam_pitch = (theta_c - CAM_PITCH_NEUTRAL) * (MAX_PPRZ / (CAM_PITCH_MAX - CAM_PITCH_NEUTRAL));
    cam_pitch = MAX_PPRZ * (theta_c / (RadOfDeg(CAM_PITCH_MAX - CAM_PITCH_NEUTRAL)) - 1.); 
  }
  else
  {
//    cam_pitch = (theta_c - CAM_PITCH_NEUTRAL) * MIN_PPRZ / (CAM_PITCH_MAX - CAM_PITCH_NEUTRAL);
    cam_pitch = MIN_PPRZ * (1. - (theta_c / (RadOfDeg(CAM_PITCH_NEUTRAL - CAM_PITCH_MIN)))); 
  }

  cam_pitch = TRIM_PPRZ(cam_pitch);

#ifdef COMMAND_HATCH
//  if (0 != use_hatch_cam) 
ap_state->commands[COMMAND_HATCH] = cam_pitch;
#endif

#if 0
printf("pitch %f, roll %f, cam_tilt %f\n", 
       (estimator_theta/M_PI)*180., (estimator_phi/M_PI)*180., (theta_c/M_PI)*180.);

printf("alt = %f\n", estimator_z - target_alt);
printf("y = %f\n", target_y-estimator_y);
printf("x = %f\n", target_x-estimator_x);
printf("servo = %f\n", cam_pitch);
#endif

#if 0
  /** Relative height of the target */
  float h = estimator_z - target_alt;

  /** Relative heading of the target (trigo) */
  float dy = target_y-estimator_y;
  float dx = target_x-estimator_x;
  float alpha = atan2(dy, dx);

  /** Projection in a horizontal plane of the distance ac-target */
  float d = sqrt (dy*dy + dx*dx);

  float d_h = d / h;
  float pseudo_psi = M_PI/2 - estimator_hspeed_dir; /** Trigo */
  float psi_alpha = - (pseudo_psi - M_PI/2 - alpha);
  phi_c = atan(d_h * cos (psi_alpha)) + estimator_phi;
  theta_c = atan(d_h * sin (psi_alpha)) - estimator_theta;
#endif
#else
  /** Relative height of the target */
  float h = estimator_z - target_alt;

  /** Relative heading of the target (trigo) */
  float dy = target_y-estimator_y;
  float dx = target_x-estimator_x;
  float alpha = atan2(dy, dx);

  /** Projection in a horizontal plane of the distance ac-target */
  float d = sqrt (dy*dy + dx*dx);

  float d_h = d / h;
  float pseudo_psi = M_PI/2 - estimator_hspeed_dir; /** Trigo */
  float psi_alpha = - (pseudo_psi - M_PI/2 - alpha);
  phi_c = atan(d_h * cos (psi_alpha)) + estimator_phi;
  theta_c = atan(d_h * sin (psi_alpha)) - estimator_theta;
#endif // POINT_CAM
}


#define MAX_DIST_TARGET 500.

void cam_manual_target( void ) {
  int16_t yaw = fbw_state->channels[RADIO_YAW];
  if (yaw > MIN_PPRZ_CAM || yaw < -MIN_PPRZ_CAM) {
    target_x += FLOAT_OF_PPRZ(yaw, 0, -20.);
    target_x = Min(target_x, MAX_DIST_TARGET + estimator_x);
    target_x = Max(target_x, -MAX_DIST_TARGET + estimator_x);
  }
  int16_t pitch = fbw_state->channels[RADIO_PITCH];
  if (pitch > MIN_PPRZ_CAM || pitch < -MIN_PPRZ_CAM) {
    target_y += FLOAT_OF_PPRZ(pitch, 0, -20.);
    target_y = Min(target_y, MAX_DIST_TARGET + estimator_y);
    target_y = Max(target_y, -MAX_DIST_TARGET + estimator_y);
  }
  target_alt = GROUND_ALT;
  cam_target();
}

void cam_waypoint_target( uint8_t wp ) {
  target_x = waypoints[wp].x;
  target_y = waypoints[wp].y;
  target_alt = GROUND_ALT;
  cam_target();
}

void cam_ac_target( uint8_t ac_id ) {
  struct ac_info_ * ac = get_ac_info(ac_id);
  target_x = ac->east;
  target_y = ac->north;
  target_alt = ac->alt;
  cam_target();
}

void cam_carrot( void ) {
  target_x = carrot_x;
  target_y = carrot_y;
  cam_target();
}

void cam_init( void ) {
}

void cam_periodic( void ) {
}

#endif // MOBILE_CAM
