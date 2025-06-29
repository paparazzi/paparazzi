/*
 * Copyright (C) 2024 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/**
 * @file stabilization_attitude_plane_pid.h
 *
 * Basic fixed-wing attitude stabilization in euler float version.
 */

#ifndef STABILIZATION_ATTITUDE_PLANE_PID_H
#define STABILIZATION_ATTITUDE_PLANE_PID_H

#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"

struct PlaneAttitudeGains {
  struct FloatVect3  p;
  struct FloatVect3  i;
  struct FloatVect3  d;
};

extern void stabilization_attitude_plane_pid_init(void);
extern void stabilization_attitude_plane_pid_enter(void);
extern void stabilization_attitude_plane_pid_run(bool in_flight, struct StabilizationSetpoint *sp, struct ThrustSetpoint *thrust, int32_t *cmd);

extern struct PlaneAttitudeGains stab_plane_gains;
extern struct FloatEulers stab_plane_att_sum_err;

#endif /* STABILIZATION_ATTITUDE_PLANE_PID_H */
