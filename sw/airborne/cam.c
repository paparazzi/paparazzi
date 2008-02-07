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

#ifdef CAM_PAN_NEUTRAL
#if (CAM_PAN_MAX == CAM_PAN_NEUTRAL)
#error CAM_PAN_MAX has to be different from CAM_PAN_NEUTRAL
#endif
#if (CAM_PAN_NEUTRAL == CAM_PAN_MIN)
#error CAM_PAN_MIN has to be different from CAM_PAN_NEUTRAL
#endif
#endif

#ifdef CAM_TILT_NEUTRAL
#if (CAM_TILT_MAX == CAM_TILT_NEUTRAL)
#error CAM_TILT_MAX has to be different from CAM_TILT_NEUTRAL
#endif
#if (CAM_TILT_NEUTRAL == CAM_TILT_MIN)
#error CAM_TILT_MIN has to be different from CAM_TILT_NEUTRAL
#endif
#endif

#define MIN_PPRZ_CAM ((int16_t)(MAX_PPRZ * 0.05))
#define DELTA_ALPHA 0.2

#ifdef CAM_PAN0
float pan_c = RadOfDeg(CAM_PAN0);
#else
float pan_c;
#endif

#ifdef CAM_TILT0
float tilt_c = RadOfDeg(CAM_TILT0);
#else
float tilt_c;
#endif

float phi_c;
float theta_c;

float target_x, target_y, target_alt;

int use_hatch_cam = 0;
int cam_mode_auto = 1;

#ifdef MOBILE_CAM

void cam_manual( void ) {
  if (pprz_mode == PPRZ_MODE_AUTO2) {
    static pprz_t cam_pan, cam_tilt;
    int16_t yaw = fbw_state->channels[RADIO_YAW];
    if (yaw > MIN_PPRZ_CAM || yaw < -MIN_PPRZ_CAM) { 
      cam_pan += FLOAT_OF_PPRZ(yaw, 0, 300.);
      cam_pan = TRIM_PPRZ(cam_pan);
    }
    int16_t pitch = fbw_state->channels[RADIO_PITCH];
    if (pitch > MIN_PPRZ_CAM || pitch < -MIN_PPRZ_CAM) {
      cam_tilt += FLOAT_OF_PPRZ(pitch, 0, 300.);
      cam_tilt = TRIM_PPRZ(cam_tilt);
    }
#ifdef COMMAND_CAM_PAN
    ap_state->commands[COMMAND_CAM_PAN] = cam_pan;
#endif
#ifdef COMMAND_CAM_TILT    
    ap_state->commands[COMMAND_CAM_TILT] = cam_tilt;
#endif    
  }
}

void cam_nadir( void ) {
  pan_c = -estimator_phi;
  tilt_c = -estimator_theta;
}

#if 0
void cam_target_trigo( void ) {
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
}
#endif

void cam_target( void ) {

  float cam_pan = 0;
  float cam_tilt = 0;

  if (cam_mode_auto != 0)
  {
    vPoint(estimator_x, estimator_y, estimator_z,
           estimator_phi, estimator_theta, estimator_hspeed_dir,
           target_x, target_y, target_alt,
           &pan_c, &tilt_c);
  }

#ifdef CAM_PAN_NEUTRAL  
  if (theta_c > RadOfDeg(CAM_PAN_NEUTRAL))
    cam_pan = MAX_PPRZ * (theta_c / (RadOfDeg(CAM_PAN_MAX - CAM_PAN_NEUTRAL)) - 1.); 
  else
    cam_pan = MIN_PPRZ * (1. - (theta_c / (RadOfDeg(CAM_PAN_NEUTRAL - CAM_PAN_MIN)))); 
#endif
  
#ifdef CAM_TILT_NEUTRAL  
  if (theta_c > RadOfDeg(CAM_TILT_NEUTRAL))
    cam_tilt = MAX_PPRZ * (theta_c / (RadOfDeg(CAM_TILT_MAX - CAM_TILT_NEUTRAL)) - 1.); 
  else
    cam_tilt = MIN_PPRZ * (1. - (theta_c / (RadOfDeg(CAM_TILT_NEUTRAL - CAM_TILT_MIN)))); 
#endif

  cam_pan = TRIM_PPRZ(cam_pan);
  cam_tilt = TRIM_PPRZ(cam_tilt);

#ifdef COMMAND_HATCH
//  if (0 != use_hatch_cam) 
  ap_state->commands[COMMAND_HATCH] = cam_tilt;
#endif
#ifdef COMMAND_CAM_PAN
    ap_state->commands[COMMAND_CAM_PAN] = cam_pan;
#endif
#ifdef COMMAND_CAM_TILT
    ap_state->commands[COMMAND_CAM_TILT] = cam_tilt;
#endif

  /* not true in all camera mount cases */
  phi_c = pan_c;
  theta_c = tilt_c;
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
