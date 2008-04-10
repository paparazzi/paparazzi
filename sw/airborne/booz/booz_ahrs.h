/*
 * $Id$
 *  
 * Copyright (C) 2008 Antoine Drouin
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

#ifndef BOOZ_AHRS_H
#define BOOZ_AHRS_H

#include "std.h"

#define BOOZ_AHRS_STATUS_UNINIT  0
#define BOOZ_AHRS_STATUS_RUNNING 1
#define BOOZ_AHRS_STATUS_CRASHED 2

extern uint8_t booz_ahrs_status;

extern FLOAT_T booz_ahrs_phi; 
extern FLOAT_T booz_ahrs_theta; 
extern FLOAT_T booz_ahrs_psi;

extern FLOAT_T booz_ahrs_p;
extern FLOAT_T booz_ahrs_q;
extern FLOAT_T booz_ahrs_r;

extern FLOAT_T booz_ahrs_bp;
extern FLOAT_T booz_ahrs_bq;
extern FLOAT_T booz_ahrs_br;

extern FLOAT_T booz_ahrs_measure_phi;
extern FLOAT_T booz_ahrs_measure_theta;
extern FLOAT_T booz_ahrs_measure_psi;

extern void booz_ahrs_init(void);
extern void booz_ahrs_start( const float* accel, const float* gyro, const float* mag);
/*extern void booz_ahrs_run(const float* accel, const float* gyro, const float* mag);*/

extern void booz_ahrs_predict( const float* gyro );
extern void booz_ahrs_update_accel( const float* accel);
extern void booz_ahrs_update_mag( const float* mag );


//#define __AHRS_IMPL_HEADER(_typ, _ext) _t##_ext
//#define _AHRS_IMPL_HEADER(_typ, _ext) __AHRS_IMPL_HEADER(_typ, _ext)
//#define AHRS_IMPL_HEADER(_typ) _AHRS_IMPL_HEADER(_typ, ".h")
//#error BOOZ_AHRS_TYPE
//#include \"AHRS_IMPL_HEADER(BOOZ_AHRS_TYPE)\"

#define BOOZ_AHRS_MULTITILT   0
#define BOOZ_AHRS_QUATERNION  1
#define BOOZ_AHRS_EULER       2
#define BOOZ_AHRS_COMP_FILTER 3

#if defined  BOOZ_AHRS_TYPE && BOOZ_AHRS_TYPE == BOOZ_AHRS_MULTITILT
#include "ahrs_multitilt.h"
#elif defined BOOZ_AHRS_TYPE && BOOZ_AHRS_TYPE == BOOZ_AHRS_QUATERNION
#include "ahrs_quat_fast_ekf.h"
#elif defined BOOZ_AHRS_TYPE && BOOZ_AHRS_TYPE == BOOZ_AHRS_EULER
#include "ahrs_euler_fast_ekf.h"
#elif defined BOOZ_AHRS_TYPE && BOOZ_AHRS_TYPE == BOOZ_AHRS_COMP_FILTER
#include "ahrs_comp_filter.h"
#endif


#define WRAP(x,a) { while (x > a) x -= 2 * a; while (x <= -a) x += 2 * a;}

#define PhiOfAccel(_phi, _accel) {			\
    _phi =  atan2(-_accel[AXIS_Y], -_accel[AXIS_Z]);	\
  }

#define ThetaOfAccel(_theta, _accel) {			\
    const float g2 =					\
      _accel[AXIS_X]*_accel[AXIS_X] +			\
      _accel[AXIS_Y]*_accel[AXIS_Y] +			\
      _accel[AXIS_Z]*_accel[AXIS_Z];			\
    _theta = asin( accel[AXIS_X] / sqrt( g2 ) );	\
  }

#define PsiOfMag(_psi, _mag) {			        \
    const float cphi   = cos( booz_ahrs_phi );		\
    const float sphi   = sin( booz_ahrs_phi );		\
    const float ctheta = cos( booz_ahrs_theta );	\
    const float stheta = sin( booz_ahrs_theta );	\
						        \
    const float mn =					\
      ctheta*      _mag[AXIS_X]+			\
      sphi*stheta* _mag[AXIS_Y]+			\
      cphi*stheta* _mag[AXIS_Z];			\
    const float me =					\
      /*    0*     mag[AXIS_X]+ */			\
      cphi*  mag[AXIS_Y]+				\
      -sphi* mag[AXIS_Z];				\
    _psi = -atan2( me, mn );				\
  }

#endif /* BOOZ_AHRS_H */
