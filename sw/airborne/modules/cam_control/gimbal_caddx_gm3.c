/*
 * Copyright (C) 2025 Julia Cabarbaye <julia.cabarbaye1@gmail.com>
 *               2025 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file "modules/cam_control/gimbal_caddx_gm3.c"
 * @author Julia Cabarbaye <julia.cabarbaye1@gmail.com>
 * caddx gm3 gimbal control sbus
 */

#include "modules/cam_control/gimbal_caddx_gm3.h"
#include "modules/cam_control/cam_gimbal.h"
#include "modules/actuators/actuators.h"
#include "generated/airframe.h"
#include "modules/datalink/datalink.h"
#include "pprzlink/dl_protocol.h"   // datalink messages

// mechanical characteristics
#define GIMBAL_CADDX_PAN_MAX  RadOfDeg(160.f)
#define GIMBAL_CADDX_TILT_MAX RadOfDeg(120.f)
#define GIMBAL_CADDX_ROLL_MAX RadOfDeg(60.f)
#define GIMBAL_CADDX_TILT_OFFSET RadOfDeg(15.f)


float gimbal_caddx_gm3_roll;

/** Compute pan and tilt angle for the 3-axis gimbal CaddX GM3
 *
 * The rotations from gimbal to camera frame are: pan (z), roll (x), tilt (y)
 * In addition, an extra mechanical offset exists along the tilt axis between the pan and roll
 * rotations. Considering the wide angle camera and the pain to inverse the resulting equation,
 * this angle is neglected.
 * Therefore, we have:
 * ->  sin(tilt) = - uz / cos(roll)
 * ->  tan(pan) = (uy / ux) - sin(roll)*tan(tilt)
 */
static void gimbal_caddx_compute_angles(struct FloatVect3 dir, float *pan, float *tilt)
{
  float roll = gimbal_caddx_gm3_roll;
  BoundAbs(roll, GIMBAL_CADDX_ROLL_MAX);
  *tilt = asinf(- dir.z / cosf(roll));
  *pan = atan2f(dir.y - sinf(roll)*tanf(*tilt)*dir.x, dir.x);
}

void gimbal_caddx_gm3_init(void)
{
  // Set the cam control with the specific parameter of CaddX GM3 gimble
  cam_gimbal_setup_angles(&cam_gimbal,
      GIMBAL_CADDX_PAN_MAX, -GIMBAL_CADDX_PAN_MAX,
      GIMBAL_CADDX_TILT_MAX, -GIMBAL_CADDX_TILT_MAX);
  cam_gimbal_set_angles_callback(&cam_gimbal, gimbal_caddx_compute_angles);

  // Set mode
#ifdef SERVO_GIMBAL_CADDX_MODE_NEUTRAL
  ActuatorSet(GIMBAL_CADDX_MODE, MIN_PPRZ);
#endif
#ifdef SERVO_GIMBAL_CADDX_SENS_NEUTRAL
  ActuatorSet(GIMBAL_CADDX_SENS, 0);
#endif

  gimbal_caddx_gm3_roll = 0.f;
#ifdef SERVO_GIMBAL_CADDX_ROLL_NEUTRAL
  // GM2 model can be used as a GM3 without roll
  ActuatorSet(GIMBAL_CADDX_ROLL, 0);
#endif
}

void gimbal_caddx_gm3_periodic(void)
{
#ifdef SERVO_GIMBAL_CADDX_ROLL_NEUTRAL
  float roll_cmd = gimbal_caddx_gm3_roll * MAX_PPRZ / GIMBAL_CADDX_ROLL_MAX;
  ActuatorSet(GIMBAL_CADDX_ROLL, roll_cmd);
#endif
}

