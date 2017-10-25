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

static abi_event range_sensor_array_ev;
static void range_sensor_array_cb(uint8_t sender_id __attribute__((unused)), int16_t range_array_size,
    uint16_t *range_array, float *orientation_array)
{
  struct FloatEulers body_to_sensors_eulers;
  struct FloatVect3 ff_vel_body = {0.f, 0.f, 0.f};
  float range;

  // loop through the range sensor array
  for (int16_t i = 0; i < range_array_size; i++) {
    // Get the orientation  and value of the range array measurement (per sensor)
    body_to_sensors_eulers.phi = orientation_array[3 * i];
    body_to_sensors_eulers.theta = orientation_array[3 * i + 1];
    body_to_sensors_eulers.psi = orientation_array[3 * i + 2];
    range = (float)range_array[i] / 1000.;

    // Calculate forcefield for that one measurement
    range_forcefield_update(&ff_vel_body, range, &body_to_sensors_eulers,
                            range_forcefield_param.inner_limit, range_forcefield_param.outer_limit,
                            range_forcefield_param.min_vel, range_forcefield_param.max_vel);
  }

  AbiSendMsgRANGE_FORCEFIELD(RANGE_FORCEFIELD_ID, ff_vel_body.x, ff_vel_body.y, ff_vel_body.z);
}


void range_forcefield_init(void)
{
  range_forcefield_param.inner_limit = RANGE_FORCEFIELD_INNER_LIMIT;
  range_forcefield_param.outer_limit = RANGE_FORCEFIELD_OUTER_LIMIT;
  range_forcefield_param.min_vel = RANGE_FORCEFIELD_MIN_VEL;
  range_forcefield_param.max_vel = RANGE_FORCEFIELD_MAX_VEL;

  AbiBindMsgRANGE_SENSOR_ARRAY(RANGE_FORCEFIELD_RECIEVE_ID, &range_sensor_array_ev, range_sensor_array_cb);
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
void range_forcefield_update(struct FloatVect3 *ff_vel_body, float range, struct FloatEulers *body_to_sensors_eulers,
                             float ff_inner_limit, float ff_outer_limit, float min_vel, float max_vel)
{
  // Velocity commands
  float avoid_command = 0.f;

  // Calculate avoidance velocity
  if (range < 0.001) {
    //do nothing
  } else if (range < ff_inner_limit) {
    avoid_command = -max_vel;
  } else if (range < ff_outer_limit) {
    // Linear
    avoid_command = -(max_vel - min_vel) * (ff_outer_limit - range)
                    / (ff_outer_limit - ff_inner_limit);
  }

  struct FloatRMat range_sensor_to_body;
  float_rmat_of_eulers(&range_sensor_to_body, body_to_sensors_eulers);

  struct FloatVect3 quad_sensor_avoid_vel = {avoid_command, 0, 0};
  struct FloatVect3 quad_body_avoid_vel;
  float_rmat_transp_vmult(&quad_body_avoid_vel, &range_sensor_to_body, &quad_sensor_avoid_vel);

  // Add velocity forcefield to previous values (for balancing the command)
  ff_vel_body->x += quad_body_avoid_vel.x;
  ff_vel_body->y += quad_body_avoid_vel.y;
  ff_vel_body->z += quad_body_avoid_vel.z;
}
