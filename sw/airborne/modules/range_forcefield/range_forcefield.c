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

#include "subsystems/abi.h"

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
static struct FloatVect3 ff_nearest_obs;

static abi_event obstacle_ev;
static void obstacle_cb(uint8_t sender_id __attribute__((unused)), float range, float azimuth,
    float bearing)
{
  static struct FloatEulers body_to_sensors_eulers;
  static struct FloatVect3 ff_vel_body;

  // loop through the range sensor array
  // Get the orientation  and value of the range array measurement (per sensor)
  body_to_sensors_eulers.phi = 0;
  body_to_sensors_eulers.theta = azimuth;
  body_to_sensors_eulers.psi = bearing;

  // Calculate forcefield for that one measurement
  range_forcefield_update(&ff_vel_body, range, &body_to_sensors_eulers, &range_forcefield_param);

  AbiSendMsgRANGE_FORCEFIELD(RANGE_FORCEFIELD_ID, ff_vel_body.x, ff_vel_body.y, ff_vel_body.z);
}

void range_forcefield_init(void)
{
  range_forcefield_param.inner_limit = RANGE_FORCEFIELD_INNER_LIMIT;
  range_forcefield_param.outer_limit = RANGE_FORCEFIELD_OUTER_LIMIT;
  range_forcefield_param.min_vel = RANGE_FORCEFIELD_MIN_VEL;
  range_forcefield_param.max_vel = RANGE_FORCEFIELD_MAX_VEL;

  ff_nearest_obs.x = RANGE_FORCEFIELD_OUTER_LIMIT;
  ff_nearest_obs.y = RANGE_FORCEFIELD_OUTER_LIMIT;
  ff_nearest_obs.z = RANGE_FORCEFIELD_OUTER_LIMIT;

  AbiBindMsgOBSTACLE_DETECTION(RANGE_FORCEFIELD_RECIEVE_ID, &obstacle_ev, obstacle_cb);
}

void range_forcefield_periodic(void)
{
  // apply forgetting factor to forcefield, handles if obstacle no longer in sight / updated
  ff_nearest_obs.x = ff_nearest_obs.x < RANGE_FORCEFIELD_OUTER_LIMIT ? ff_nearest_obs.x*1.1 : RANGE_FORCEFIELD_OUTER_LIMIT;
  ff_nearest_obs.y = ff_nearest_obs.y < RANGE_FORCEFIELD_OUTER_LIMIT ? ff_nearest_obs.y*1.1 : RANGE_FORCEFIELD_OUTER_LIMIT;
  ff_nearest_obs.z = ff_nearest_obs.z < RANGE_FORCEFIELD_OUTER_LIMIT ? ff_nearest_obs.z*1.1 : RANGE_FORCEFIELD_OUTER_LIMIT;
}

/* Compute the forcefield velocity command given the distance to the obstacle and the forcefield parameters
 * @param range Distance to obstacle
 * @param ff_params The forcefield settings to generate the velocity
 */
static float compute_ff_vel(float range, struct range_forcefield_param_t* ff_params)
{
  float ff_vel = 0.f;
  // Calculate avoidance velocity
  if (range < 0.001f) {
    //do nothing
  } else if (range < ff_params->inner_limit) {
    ff_vel = -ff_params->max_vel;
  } else if (range < ff_params->outer_limit) {
    // Linear
    ff_vel = -(ff_params->max_vel - ff_params->min_vel) * (ff_params->outer_limit - range)
                    / (ff_params->outer_limit - ff_params->inner_limit);
  }
  return ff_vel;
}

/* range_sensor_update_forcefield: This function adjusts the intended velocity commands in the horizontal plane
 * to move away from obstacles and walls as detected by single ray range sensors. An simple linear equation with
 * the distance measured by the range sensors and the given minimum and maximum avoid velocity, it will create a
 * velocity forcefield which can be used during a guided flight.
 *
 * @param[out] ff_vel_body, intended body fixed velocity in the y direction in [m/s]
 * @param[in] range, input range measurement
 * @param[in] body_to_sensors_eulers, euler angles of the orientation of the range sensor in question[rad]
 * @param[in] ff_inner_limit, minimum allowable distance from the obstacle in [m]
 * @param[in] ff_outer_limit, maximum allowable distance from the obstacle in [m]
 * @param[in] min_vel, minimum velocity for the forcefield generation [m/s]
 * @param[in] max_vel, maximum velocity for the forcefield generation [m/s]
 * */
void range_forcefield_update(struct FloatVect3 *ff_vel_body, float range, struct FloatEulers *body_to_sensor_eulers,
                             struct range_forcefield_param_t* ff_params)
{
  // generate vector to obstacle location based on range and rotation
  struct FloatVect3 obs_loc, range_vec = {range, 0, 0};
  struct FloatRMat range_sensor_to_body;
  float_rmat_of_eulers(&range_sensor_to_body, body_to_sensor_eulers);
  float_rmat_transp_vmult(&obs_loc, &range_sensor_to_body, &range_vec);

  // store closest distance in each axis
  ff_nearest_obs.x = obs_loc.x > ff_nearest_obs.x ? obs_loc.x : ff_nearest_obs.x;
  ff_nearest_obs.y = obs_loc.y > ff_nearest_obs.y ? obs_loc.y : ff_nearest_obs.y;
  ff_nearest_obs.z = obs_loc.z > ff_nearest_obs.z ? obs_loc.z : ff_nearest_obs.z;

  ff_vel_body->x = compute_ff_vel(ff_nearest_obs.x, ff_params);
  ff_vel_body->y = compute_ff_vel(ff_nearest_obs.y, ff_params);
  ff_vel_body->z = compute_ff_vel(ff_nearest_obs.z, ff_params);
}
