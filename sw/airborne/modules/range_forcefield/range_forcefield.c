/*
 * Copyright (C) 2017 K. N. McGuire
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

/*
 * @file "modules/range_forcefield/range_forcefield.c"
 * @author K. N. McGuire
 * This module generates a forcefield based on range sensor measurements the use of single point range sensors.
 */

#include "modules/range_forcefield/range_forcefield.h"

#include "modules/core/abi.h"

//abi for range sensors
#ifndef RANGE_FORCEFIELD_RECIEVE_ID
#define RANGE_FORCEFIELD_RECIEVE_ID ABI_BROADCAST
#endif

#ifndef RANGE_FORCEFIELD_INNER_LIMIT
#define RANGE_FORCEFIELD_INNER_LIMIT 1.0f
#endif
PRINT_CONFIG_VAR(RANGE_FORCEFIELD_INNER_LIMIT)

#ifndef RANGE_FORCEFIELD_OUTER_LIMIT
#define RANGE_FORCEFIELD_OUTER_LIMIT 1.4f
#endif
PRINT_CONFIG_VAR(RANGE_FORCEFIELD_OUTER_LIMIT)

#ifndef RANGE_FORCEFIELD_MIN_VEL
#define RANGE_FORCEFIELD_MIN_VEL 0.0f
#endif
PRINT_CONFIG_VAR(RANGE_FORCEFIELD_MIN_VEL)

#ifndef RANGE_FORCEFIELD_MAX_VEL
#define RANGE_FORCEFIELD_MAX_VEL 0.5f
#endif
PRINT_CONFIG_VAR(RANGE_FORCEFIELD_MAX_VEL)

struct range_forcefield_param_t range_forcefield_param;
static struct FloatVect3 ff_nearest_obs_pos;  // nearest obstacle on positive axis
static struct FloatVect3 ff_nearest_obs_neg;  // nearest obstacle on negative axis

static float compute_ff_vel(float range);
static void store_min_dist(float pos, float *nearest_pos, float range);

static abi_event obstacle_ev;
static void obstacle_cb(uint8_t sender_id __attribute__((unused)), float range, float azimuth,
    float bearing)
{
  static struct FloatEulers body_to_sensors_eulers;

  body_to_sensors_eulers.phi = 0;
  body_to_sensors_eulers.theta = azimuth;
  body_to_sensors_eulers.psi = bearing;

  range_forcefield_update(range, &body_to_sensors_eulers);
}

void range_forcefield_init(void)
{
  range_forcefield_param.inner_limit = RANGE_FORCEFIELD_INNER_LIMIT;
  range_forcefield_param.outer_limit = RANGE_FORCEFIELD_OUTER_LIMIT;
  range_forcefield_param.min_vel = RANGE_FORCEFIELD_MIN_VEL;
  range_forcefield_param.max_vel = RANGE_FORCEFIELD_MAX_VEL;

  ff_nearest_obs_pos.x = RANGE_FORCEFIELD_OUTER_LIMIT;
  ff_nearest_obs_pos.y = RANGE_FORCEFIELD_OUTER_LIMIT;
  ff_nearest_obs_pos.z = RANGE_FORCEFIELD_OUTER_LIMIT;

  ff_nearest_obs_neg.x = -RANGE_FORCEFIELD_OUTER_LIMIT;
  ff_nearest_obs_neg.y = -RANGE_FORCEFIELD_OUTER_LIMIT;
  ff_nearest_obs_neg.z = -RANGE_FORCEFIELD_OUTER_LIMIT;

  AbiBindMsgOBSTACLE_DETECTION(RANGE_FORCEFIELD_RECIEVE_ID, &obstacle_ev, obstacle_cb);
}

/* Compute range forcefield and send abi message
 *
 */
void range_forcefield_periodic(void)
{
  static struct FloatVect3 ff_vel_body;
  ff_vel_body.x = compute_ff_vel(ff_nearest_obs_pos.x) - compute_ff_vel(fabsf(ff_nearest_obs_neg.x));
  ff_vel_body.y = compute_ff_vel(ff_nearest_obs_pos.y) - compute_ff_vel(fabsf(ff_nearest_obs_neg.y));
  ff_vel_body.z = compute_ff_vel(ff_nearest_obs_pos.z) - compute_ff_vel(fabsf(ff_nearest_obs_neg.z));

  AbiSendMsgRANGE_FORCEFIELD(RANGE_FORCEFIELD_ID, ff_vel_body.x, ff_vel_body.y, ff_vel_body.z);

  // apply forgetting factor to forcefield, handles if obstacle no longer in sight / updated
  ff_nearest_obs_pos.x = ff_nearest_obs_pos.x < RANGE_FORCEFIELD_OUTER_LIMIT ? ff_nearest_obs_pos.x*1.1 : RANGE_FORCEFIELD_OUTER_LIMIT;
  ff_nearest_obs_pos.y = ff_nearest_obs_pos.y < RANGE_FORCEFIELD_OUTER_LIMIT ? ff_nearest_obs_pos.y*1.1 : RANGE_FORCEFIELD_OUTER_LIMIT;
  ff_nearest_obs_pos.z = ff_nearest_obs_pos.z < RANGE_FORCEFIELD_OUTER_LIMIT ? ff_nearest_obs_pos.z*1.1 : RANGE_FORCEFIELD_OUTER_LIMIT;

  ff_nearest_obs_neg.x = ff_nearest_obs_neg.x > -RANGE_FORCEFIELD_OUTER_LIMIT ? ff_nearest_obs_neg.x*1.1 : -RANGE_FORCEFIELD_OUTER_LIMIT;
  ff_nearest_obs_neg.y = ff_nearest_obs_neg.y > -RANGE_FORCEFIELD_OUTER_LIMIT ? ff_nearest_obs_neg.y*1.1 : -RANGE_FORCEFIELD_OUTER_LIMIT;
  ff_nearest_obs_neg.z = ff_nearest_obs_neg.z > -RANGE_FORCEFIELD_OUTER_LIMIT ? ff_nearest_obs_neg.z*1.1 : -RANGE_FORCEFIELD_OUTER_LIMIT;
}

/* range_sensor_update_forcefield: This stores the range sensor measurement in
 *
 * @param[in] range, input range measurement
 * @param[in] body_to_sensors_eulers, euler angles of the orientation of the range sensor in question[rad]
 * */
void range_forcefield_update(float range, struct FloatEulers *body_to_sensor_eulers)
{
  // generate vector to obstacle location based on range and rotation
  struct FloatVect3 obs_loc, range_vec = {range, 0, 0};
  struct FloatRMat range_sensor_to_body;
  float_rmat_of_eulers(&range_sensor_to_body, body_to_sensor_eulers);
  float_rmat_transp_vmult(&obs_loc, &range_sensor_to_body, &range_vec);

  // store closest distance in each axis
  store_min_dist(obs_loc.x, &(ff_nearest_obs_pos.x), range);
  store_min_dist(obs_loc.x, &(ff_nearest_obs_neg.x), range);

  store_min_dist(obs_loc.y, &(ff_nearest_obs_pos.y), range);
  store_min_dist(obs_loc.y, &(ff_nearest_obs_neg.y), range);

  store_min_dist(obs_loc.z, &(ff_nearest_obs_pos.z), range);
  store_min_dist(obs_loc.z, &(ff_nearest_obs_neg.z), range);
}

/* Compute the forcefield velocity command given the distance to the obstacle and the forcefield parameters
 * @param range Distance to obstacle
 * @param ff_params The forcefield settings to generate the velocity
 */
static float compute_ff_vel(float range)
{
  float ff_vel = 0.f;
  // Calculate avoidance velocity
  if (range < 0.001f) {
    //do nothing
  } else if (range < range_forcefield_param.inner_limit) {
    ff_vel = -range_forcefield_param.max_vel;
  } else if (range < range_forcefield_param.outer_limit) {
    // Linear
    ff_vel = -(range_forcefield_param.max_vel - range_forcefield_param.min_vel) * (range_forcefield_param.outer_limit - range)
                    / (range_forcefield_param.outer_limit - range_forcefield_param.inner_limit);
  }
  return ff_vel;
}

/* Store the minimum directional distance
 * pos directional position on axis
 * nearest_pos     pointer to store nearest position
 * range           magnitude of the range vector being stored
 */
static void store_min_dist(float pos, float *nearest_pos, float range)
{
  // check if obstacle in range for this axis
  // 1/2 of total distance vector relates to an angle of 60deg in plane
  if (fabsf(pos) >= range / 2.f)
  {
    // if obstacle should be considered in x axis, remember extremum in either direction
    if (pos > 0.f && pos < *nearest_pos)
    {
      *nearest_pos = pos;
    } else if (pos < 0.f && pos > *nearest_pos)
    {
      *nearest_pos = pos;
    }
  }
}
