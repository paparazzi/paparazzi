/*
 * $Id$
 *
 * Copyright (C) 2010 The Paparazzi Team
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
 */

#ifndef AHRS_FLOAT_CMPL_RMAT
#define AHRS_FLOAT_CMPL_RMAT

struct AhrsFloatCmplRmat {
  struct FloatRates gyro_bias;
  struct FloatRates rate_correction;
  /* for gravity correction during coordinated turns */
  struct FloatVect3 est_ltp_speed;

  /*
     Holds float version of IMU alignement
     in order to be able to run against the fixed point
     version of the IMU
  */
  struct FloatQuat body_to_imu_quat;
  struct FloatRMat body_to_imu_rmat;
};

extern struct AhrsFloatCmplRmat ahrs_impl;


#endif /* AHRS_FLOAT_CMPL_RMAT */
