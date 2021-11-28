/*
 * Copyright (C) 2021 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file "modules/meteo/wind_estimation_quadrotor.h"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Wind estimation from quadrotor motion
 */

#ifndef WIND_ESTIMATION_QUADROTOR_H
#define WIND_ESTIMATION_QUADROTOR_H

struct wind_estimation_quadrotor_params {
  float Q_va;   ///< model noise on airspeed
  float Q_w;    ///< model noise on wind
  float R;      ///< measurement noise (ground speed)
};

extern struct wind_estimation_quadrotor_params we_quad_params;

extern void wind_estimation_quadrotor_init(void);
extern void wind_estimation_quadrotor_periodic(void);
extern void wind_estimation_quadrotor_stop(void);
extern void wind_estimation_quadrotor_start(void);
extern void wind_estimation_quadrotor_report(void);

extern float wind_estimation_quadrotor_SetQva(float Q_va);
extern float wind_estimation_quadrotor_SetQw(float Q_w);
extern float wind_estimation_quadrotor_SetR(float R);

#endif  // WIND_ESTIMATION_QUADROTOR_H
