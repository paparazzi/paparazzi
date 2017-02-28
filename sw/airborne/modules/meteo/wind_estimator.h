/*
 * Copyright (C) 2016 Johan Maurin, Gautier Hattenberger
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file "modules/meteo/wind_estimator.h"
 *
 * Wind Estimator based on generated library from Matlab
 *
 * Original Simulink files available at https://github.com/enacuavlab/UKF_Wind_Estimation
 */

#ifndef WIND_ESTIMATOR_H
#define WIND_ESTIMATOR_H

#include "std.h"
#include "math/pprz_algebra_float.h"

struct WindEstimator {
  struct FloatVect3 airspeed;   ///< airspeed vector in body frame
  struct FloatVect3 wind;       ///< wind vector in NED frame
  bool data_available;          ///< new data available
  bool reset;                   ///< reset filter flag

  // filter parameters
  float r_gs;       ///< noise associated to ground speed measurement
  float r_va;       ///< noise associated to airspeed norm measurement
  float r_aoa;      ///< noise associated to angle of attack measurement
  float r_ssa;      ///< noise associated to sideslip angle measurement
  float q_va;       ///< noise associated to airspeed vector model
  float q_wind;     ///< noise associated to wind vector model
  float q_va_scale; ///< noise associated to airspeed scale factor model
};

extern struct WindEstimator wind_estimator;

extern void wind_estimator_init(void);
extern void wind_estimator_periodic(void);
extern void wind_estimator_event(void);

// paramenters settings handler
extern void wind_estimator_Set_R_GS(float _v);
extern void wind_estimator_Set_R_VA(float _v);
extern void wind_estimator_Set_R_AOA(float _v);
extern void wind_estimator_Set_R_SSA(float _v);
extern void wind_estimator_Set_Q_VA(float _v);
extern void wind_estimator_Set_Q_WIND(float _v);
extern void wind_estimator_Set_Q_VA_SCALE(float _v);

#endif

