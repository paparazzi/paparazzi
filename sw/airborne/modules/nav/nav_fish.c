/*
 * Copyright (C) 2020 Adjiri Adam <adam.adjiri@etu.isae-ensma.fr>
 *                    Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file "modules/nav/navigation_fish.c"
 * @author Adjiri Adam <adam.adjiri@etu.isae-ensma.fr>
 * Bio-inspired swarm navigation
 *
 * Based on:
 * Calovi DS, Litchinko A, Lecheval V, Lopez
 * U, Pérez Escudero A, Chaté H, et al. (2018)
 * Disentangling and modeling interactions in fish
 * with burst-and-coast swimming reveal distinct
 * alignment and attraction behaviors. PLoS Comput
 * Biol 14(1): e1005933.
 * https://doi.org/10.1371/journal.pcbi.1005933
 */

#include "modules/nav/nav_fish.h"
#include "modules/multi/traffic_info.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "state.h"
#include "autopilot.h"
#include "autopilot_guided.h"
#include "subsystems/navigation/waypoints.h"
#include "math/pprz_geodetic_float.h"
#include "generated/flight_plan.h"
#include <math.h>
#include "subsystems/datalink/telemetry.h"

#include <stdio.h>
#include <stdlib.h>

/**
 * Default parameters
 */

#ifndef NAV_FISH_BODY_LENGTH
#define NAV_FISH_BODY_LENGTH 0.5f
#endif

#ifndef NAV_FISH_MAXVELOCITY
#define NAV_FISH_MAXVELOCITY 0.5f
#endif

#ifndef NAV_FISH_MINVELOCITY
#define NAV_FISH_MINVELOCITY 0.1f
#endif

#ifndef NAV_FISH_MIND2D
#define NAV_FISH_MIND2D 1.f
#endif

#ifndef NAV_FISH_FLUCT
#define NAV_FISH_FLUCT 0.1f
#endif

#ifndef NAV_FISH_EW1
#define NAV_FISH_EW1 0.7f
#endif

#ifndef NAV_FISH_EW2
#define NAV_FISH_EW2 0.f
#endif

#ifndef NAV_FISH_ALPHA
#define NAV_FISH_ALPHA 0.6f
#endif

#ifndef NAV_FISH_YW
#define NAV_FISH_YW 0.8f
#endif

#ifndef NAV_FISH_LW
#define NAV_FISH_LW (2.5f*NAV_FISH_BODY_LENGTH)
#endif

#ifndef NAV_FISH_ALPHA_REP
#define NAV_FISH_ALPHA_REP 1.f
#endif

#ifndef NAV_FISH_YATT
#define NAV_FISH_YATT 0.4f
#endif

#ifndef NAV_FISH_LATT
#define NAV_FISH_LATT 3.f
#endif

#ifndef NAV_FISH_D0ATT
#define NAV_FISH_D0ATT 1.5f
#endif

#ifndef NAV_FISH_YALI
#define NAV_FISH_YALI 0.15f
#endif

#ifndef NAV_FISH_LALI
#define NAV_FISH_LALI 3.f
#endif

#ifndef NAV_FISH_D0ALI
#define NAV_FISH_D0ALI 1.f
#endif

#ifndef NAV_FISH_ALT
#define NAV_FISH_ALT 2.0f
#endif

#ifndef NAV_FISH_WALL_DISTANCE
#define NAV_FISH_WALL_DISTANCE 10.f
#endif

#ifndef NAV_FISH_STRATEGY
#define NAV_FISH_STRATEGY 0
#endif

#ifndef NAV_FISH_TRYATT
#define NAV_FISH_TRYATT 0.2f
#endif

#ifndef NAV_FISH_TRLATT
#define NAV_FISH_TRLATT 3.f
#endif

#ifndef NAV_FISH_TRYALI
#define NAV_FISH_TRYALI 0.25f
#endif

#ifndef NAV_FISH_TRLALI
#define NAV_FISH_TRLALI 2.f
#endif

static float distance_wall = NAV_FISH_WALL_DISTANCE;
struct NavFishParams nav_fish_params;
// shortname to params
#define nfp nav_fish_params

//state data
struct NavFish {
  float heading;      ///< heading command
  float step_size;    ///< step size
  float r_w;          ///< distance to wall
  float f_w;          ///< intensity of wall effect
  float theta_w;      ///< angle to wall
  float f_fluct;      ///< fluctuation effect
  float f_wall;       ///< wall effect
  float f_ali;        ///< alignement effect
  float f_att;        ///< attraction effect
  float velocity;     ///< current velocity
};

struct NavFish nav_fish;

/** initialization of parameters and state variables
*/
void nav_fish_init(void)
{
  nav_fish_params.max_velocity = NAV_FISH_MAXVELOCITY;
  nav_fish_params.min_velocity = NAV_FISH_MINVELOCITY;
  nav_fish_params.min_d2d      = NAV_FISH_MIND2D;
  nav_fish_params.fluct        = NAV_FISH_FLUCT;
  nav_fish_params.alpha        = NAV_FISH_ALPHA;
  nav_fish_params.e_w1         = NAV_FISH_EW1;
  nav_fish_params.e_w2         = NAV_FISH_EW2;
  nav_fish_params.y_w          = NAV_FISH_YW;
  nav_fish_params.l_w          = NAV_FISH_LW;
  nav_fish_params.alpha_rep    = NAV_FISH_ALPHA_REP;
  nav_fish_params.y_att        = NAV_FISH_YATT;
  nav_fish_params.l_att        = NAV_FISH_LATT;
  nav_fish_params.d0_att       = NAV_FISH_D0ATT;
  nav_fish_params.y_ali        = NAV_FISH_YALI;
  nav_fish_params.l_ali        = NAV_FISH_LALI;
  nav_fish_params.d0_ali       = NAV_FISH_D0ALI;
  nav_fish_params.alt          = NAV_FISH_ALT;
  nav_fish_params.tr_y_att     = NAV_FISH_TRYATT;
  nav_fish_params.tr_l_att     = NAV_FISH_TRLATT;
  nav_fish_params.tr_y_ali     = NAV_FISH_TRYALI;
  nav_fish_params.tr_l_ali     = NAV_FISH_TRLALI;
  nav_fish_params.strategy     = NAV_FISH_STRATEGY;

  nav_fish.heading = 0.f;
  nav_fish.step_size = NAV_FISH_BODY_LENGTH;
  nav_fish.r_w = 0.f;
  nav_fish.f_w = 0.f;
  nav_fish.theta_w = 0.f;
  nav_fish.f_fluct = 0.f;
  nav_fish.f_wall = 0.f;
  nav_fish.f_ali = 0.f;
  nav_fish.f_att = 0.f;
  nav_fish.velocity = 0.5f;
}


static inline void send_swarm_message(void)
{
  DOWNLINK_SEND_SWARM_FISH(DefaultChannel, DefaultDevice,
      &nav_fish.heading,
      &nav_fish.step_size,
      &nav_fish.r_w,
      &nav_fish.f_w,
      &nav_fish.theta_w,
      &nav_fish.f_fluct,
      &nav_fish.f_wall,
      &nav_fish.f_ali,
      &nav_fish.f_att);
}

/** sign function
 *
 * @param number x
 * @return sign of x
 */
static float sign(float x)
{
  if(x >= 0.f) {
    return 1.f;
  } else {
    return -1.f;
  }
}

/** Gaussian random number generator with mean =0 and invariance =1 using Box-Muller method
 * @return random number following a normal distribution
 */
static float normal_random_gen(void)
{
  float random1 = ((float)rand()) / ((float)(RAND_MAX));
  float random2 = ((float)rand()) / ((float)(RAND_MAX));
  return cosf(2.f*M_PI * random1) * sqrtf(-2.f*logf(random2));
}

/** Calculates distance between the uav and wall
 *
 * @param pos ENU coordinates
 * @return distance to circular wall
 */
static float distance_to_wall(struct EnuCoor_f *pos)
{
  float x_home = waypoint_get_x(WP_HOME);
  float y_home = waypoint_get_y(WP_HOME);
  float dist = distance_wall - sqrtf(((pos->x - x_home) * (pos->x - x_home)) + ((pos->y - y_home) * (pos->y - y_home)));
  if (dist < 0.f) {
    return 0.f;
  }
  return dist;
}

/** calculates the relative orientation too the wall
 *
 * @param pos ENU coordinates
 * @param psi angle from north clockwise
 * @return angle to circular wall
 */
static float angle_to_wall(struct EnuCoor_f *pos, float psi)
{
  float dir = M_PI_2 - psi;
  float theta = atan2f(pos->y, pos->x);
  float delta = dir - theta;
  FLOAT_ANGLE_NORMALIZE(delta);
  return delta;
}

/** calculates the distance between two uavs
 *
 * @param pos ENU coordinates of the first drone
 * @param other ENU coordinates of the second drone
 * @return the distance between two uavs from their given position
 */
static float distance_drone_to_drone(struct EnuCoor_f *pos, struct EnuCoor_f *other)
{
  struct EnuCoor_f diff;
  VECT3_DIFF(diff, *other, *pos);
  return sqrtf(VECT2_NORM2(diff));
}

/** calculates a uav's viewing angle on another uav
 * @param pos ENU coordinates of the viewer drone
 * @param other ENU coordinates of the viewed drone
 * @param psi heading of the viewer drone
 * @return viewing angle
 */
static float viewing_angle(struct EnuCoor_f *pos, struct EnuCoor_f *other, float psi)
{
  struct EnuCoor_f diff;
  VECT3_DIFF(diff, *other, *pos);
  float dir = 0.f;
  if (fabsf(diff.x) < 1e-5f) {
    dir = sign(diff.y) * M_PI_2;
  } else {
    dir = atanf(diff.y/diff.x);
  }
  return (sign(diff.x)*M_PI_2) - psi - dir;
}

/** calculates difference between two headings
 * @param psi first heading
 * @param psi_other second heading
 * @return the difference between the two headings
 */
static float delta_phi(float psi, float psi_other)
{
  float dp = psi_other - psi;
  FLOAT_ANGLE_NORMALIZE(dp);
  return dp;
}

/** calculates the influence of a uav on a neighbor of his
 * @param pos ENU coordinates of the influenced uav
 * @param other ENU coordinates of the influencing uav
 * @param psi heading of the influenced uav
 * @param psi_other heading of the influencing uav
 * @return influence
 */
static float neighbor_influence(struct EnuCoor_f *pos, struct EnuCoor_f *other, float psi , float psi_other)
{
  float view = viewing_angle(pos, other, psi);
  float d2d = distance_drone_to_drone(pos, other);
  float d_phi = delta_phi(psi, psi_other);
  float tmp_ali = d2d / nfp.l_ali;
  tmp_ali = nfp.y_ali * sinf(d_phi) * ((d2d + nfp.d0_ali) / nfp.l_ali) * expf(-tmp_ali * tmp_ali);
  float amplifier = 1.f;
  if (d2d < nfp.d0_att) {
    amplifier = expf(nfp.alpha_rep * ((nfp.d0_att - d2d)/(nfp.d0_att)));
  }
  float tmp_att = d2d / nfp.l_att;
  tmp_att = nfp.y_att * (((d2d - nfp.d0_att) / nfp.l_att) / (1.f + tmp_att * tmp_att)) * sinf(view) * amplifier;
  return fabs(tmp_att + tmp_ali);
}

/** calculates the attraction effect between two uavs
 * @param pos ENU coordinates of the current uav
 * @param other ENU coordinates of the neighbor uav
 * @param psi heading of the current uav
 * @return attraction
 */
static float neighbor_attraction(struct EnuCoor_f *pos, struct EnuCoor_f *other, float psi)
{
  float view = viewing_angle(pos, other, psi);
  float d2d = distance_drone_to_drone(pos, other);

  if(d2d <= nfp.min_d2d) {
    if (view > -M_PI_2 && view <= M_PI_2) {
      nav_fish.velocity = nav_fish.velocity - nfp.min_velocity;
    } else {
      nav_fish.velocity = nav_fish.velocity + nfp.min_velocity;
    }
  }
  float amplifier = 1.f;
  if (d2d < nfp.d0_att) {
    amplifier = expf(nfp.alpha_rep * ((nfp.d0_att - d2d) / nfp.d0_att));
  }
  float tmp_att = d2d / nfp.l_att;
  tmp_att = nfp.y_att * (((d2d - nfp.d0_att) / nfp.l_att) / (1.f + tmp_att * tmp_att)) * sinf(view) * amplifier;
  return tmp_att;
}

/** calculates the alignement effect between two uavs
 * @param pos ENU coordinates of the current uav
 * @param other ENU coordinates of the neighbor uav
 * @param psi heading of the current uav
 * @param psi_other heading of the neighbor uav
 * @return alignement
 */
static float neighbor_alignement(struct EnuCoor_f *pos, struct EnuCoor_f *other, float psi , float psi_other)
{
  float d2d = distance_drone_to_drone(pos, other);
  float d_phi = delta_phi(psi, psi_other);
  float tmp_ali = d2d / nfp.l_ali;
  tmp_ali = nfp.y_ali * sinf(d_phi) * ((d2d + nfp.d0_ali) / nfp.l_ali) * expf(-tmp_ali * tmp_ali);
  return tmp_ali;
}

//static float perpendicular_distance(float x, float y, float sx, float sy, float dx, float dy)
//{
//  float agent = (dy * (y - sy) + dx * (x - sx)) / (dx * dx + dy * dy);
//  float x_new = sx + dx * agent;
//  float y_new = sy + dy * agent;
//  return sqrtf((x - x_new) * (x - x_new) + (y - y_new) * (y - y_new));
//}

/** calculates new variation of the heading for the uav based on current state
 * @return heading variation
 */
static float calculate_new_heading(void)
{
  struct EnuCoor_f *pos = stateGetPositionEnu_f();
  float psi = stateGetNedToBodyEulers_f()->psi;

  // compute distance and angle to wall
  nav_fish.r_w = distance_to_wall(pos);
  nav_fish.theta_w = angle_to_wall(pos, psi);

  // trajectory attraction and alignement (not ready)
  //float dx = x_finish - x_start;
  //float dy = y_finish - y_start;
  //float alpha = atan2f(dx, dy) - psi;
  //float pd = perpendicular_distance(pos->x, pos->y, x_start, y_start, dx, dy);
  //float beta = (sign(x_finish - pos->x)*M_PI_2) - psi - atanf((y_finish - pos->y)/(x_finish - pos->x)) ;
  //float trajectory = (nfp.tr_y_ali * sinf(alpha) * expf(nfp.tr_l_ali - pd)) + (nfp.tr_y_att * sinf(beta) * expf((pd - (3 * nfp.tr_l_att)) / (nfp.tr_l_att + pd)));

  // compute fluctuation and wall reaction
  float fw = expf(-powf(nav_fish.r_w / nfp.l_w, 2.f));
  float ow = (-1.0f)*sinf(nav_fish.theta_w) * (1.f + nfp.e_w1 * cosf(nav_fish.theta_w) + nfp.e_w2 * cosf(2.f * nav_fish.theta_w));
  nav_fish.f_w = fw;
  nav_fish.f_fluct = nfp.fluct * (1.f - nfp.alpha * fw) * normal_random_gen();
  nav_fish.f_wall = nfp.y_w * ow * fw;
  nav_fish.step_size = (1.1f - fw) / 2.f;

  // compute alignement and attraction with other drones
  struct EnuCoor_f *pos_focal = NULL;
  float psi_focal = 0.f;
  //uint8_t id_focal = 0;
  struct EnuCoor_f *pos_current = NULL;
  float psi_current = 0.f;
  uint8_t id_current = 0;
  float min_distance = 1000000.f;
  float max_influence = 0.f;
  nav_fish.f_ali = 0.f;
  nav_fish.f_att = 0.f;

  for (uint8_t ac = 0; ac < NB_ACS; ac++) {
    if (ti_acs[ac].ac_id == AC_ID || ti_acs[ac].ac_id == 0) { continue; }
    float delta_t = Max((int)(gps.tow - acInfoGetItow(ti_acs[ac].ac_id)) / 1000., 0.);
    if (delta_t < 1.f) {
      // compute if others position is not older than 1s
      id_current = ti_acs[ac].ac_id;
      pos_current = acInfoGetPositionEnu_f(id_current);
      psi_current = acInfoGetCourse(id_current);
      if (nfp.strategy == 2) {
        pos_focal = NULL;
        nav_fish.f_ali = nav_fish.f_ali + neighbor_alignement(pos, pos_current, psi, psi_current);
        nav_fish.f_att = nav_fish.f_att + neighbor_attraction(pos, pos_current, psi);
      }
      if (nfp.strategy == 1) {
        float influence = neighbor_influence(pos, pos_current ,psi ,psi_current);
        if (influence > max_influence) {
          max_influence = influence;
          //id_focal = id_current;
          pos_focal = pos_current;
          psi_focal = psi_current;
        }
      }
      if (nfp.strategy == 0) {
        float distance = distance_drone_to_drone(pos, pos_current);
        if (distance < min_distance) {
          min_distance = distance;
          //id_focal = id_current;
          pos_focal = pos_current;
          psi_focal = psi_current;
        }
      }
    }
  }
  if (pos_focal != NULL) {
    nav_fish.f_ali = neighbor_alignement(pos, pos_focal, psi, psi_focal);
    nav_fish.f_att = neighbor_attraction(pos, pos_focal, psi);
  }

  // compute heading variation
  float diff_heading = nav_fish.f_fluct + nav_fish.f_wall + nav_fish.f_ali + nav_fish.f_att;
  return diff_heading;
}

#define PRESCALE 16
/** runs the uav according to fish movement model using velocity control
 * @return true
 */
bool nav_fish_velocity_run(void)
{
  static int counter = PRESCALE;
  counter = counter - (1 + (int)(nav_fish.f_w * (PRESCALE - 1)));
  if (counter > 0) { return true; }
  counter = PRESCALE;
  float diff_heading = calculate_new_heading();
  float new_heading = nav_fish.heading + diff_heading;
  FLOAT_ANGLE_NORMALIZE(new_heading);
  nav_fish.heading = new_heading;
  autopilot_guided_update(GUIDED_FLAG_XY_BODY | GUIDED_FLAG_XY_VEL, nav_fish.velocity, 0.0f, -nfp.alt, nav_fish.heading);
  nav_fish.velocity = nfp.max_velocity;
#if NAV_FISH_SYNC_SEND
  send_swarm_message();
#endif
  return true;
}


