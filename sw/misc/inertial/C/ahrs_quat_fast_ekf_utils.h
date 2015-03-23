#ifndef AHRS_QUAT_FAST_EKF_UTILS_H
#define AHRS_QUAT__FAST_EKF_UTILS_H

#include "ahrs_quat_fast_ekf.h"

/*
 * Compute the five elements of the DCM that we use for our
 * rotations and Jacobians.  This is used by several other functions
 * to speedup their computations.
 */
#define AFE_DCM_OF_QUAT() {				\
    afe_dcm00 = 1.0-2*(afe_q2*afe_q2 + afe_q3*afe_q3);	\
    afe_dcm01 =     2*(afe_q1*afe_q2 + afe_q0*afe_q3);	\
    afe_dcm02 =     2*(afe_q1*afe_q3 - afe_q0*afe_q2);	\
    afe_dcm12 =     2*(afe_q2*afe_q3 + afe_q0*afe_q1);	\
    afe_dcm22 = 1.0-2*(afe_q1*afe_q1 + afe_q2*afe_q2);	\
  }
/*
 * Compute Euler angles from our DCM.
 */
#define AFE_PHI_OF_DCM()   { afe_phi = atan2( afe_dcm12, afe_dcm22 );}
#define AFE_THETA_OF_DCM() { afe_theta = -asin( afe_dcm02 );}
#define AFE_PSI_OF_DCM()   { afe_psi = atan2( afe_dcm01, afe_dcm00 );}
#define AFE_EULER_OF_DCM() { AFE_PHI_OF_DCM(); AFE_THETA_OF_DCM(); AFE_PSI_OF_DCM()}

/*
 * initialise the quaternion from the set of eulers
 */
#define AFE_QUAT_OF_EULER() {			                             \
    const FLOAT_T phi2     = afe_phi / 2.0;	                             \
    const FLOAT_T theta2   = afe_theta / 2.0;	                             \
    const FLOAT_T psi2     = afe_psi / 2.0;	                             \
						                             \
    const FLOAT_T sinphi2   = sin( phi2 );	                             \
    const FLOAT_T cosphi2   = cos( phi2 );	                             \
						                             \
    const FLOAT_T sintheta2 = sin( theta2 );	                             \
    const FLOAT_T costheta2 = cos( theta2 );	                             \
						                             \
    const FLOAT_T sinpsi2   = sin( psi2 );	                             \
    const FLOAT_T cospsi2   = cos( psi2 );	                             \
    									     \
    afe_q0 =  cosphi2 * costheta2 * cospsi2 + sinphi2 * sintheta2 * sinpsi2; \
    afe_q1 = -cosphi2 * sintheta2 * sinpsi2 + sinphi2 * costheta2 * cospsi2; \
    afe_q2 =  cosphi2 * sintheta2 * cospsi2 + sinphi2 * costheta2 * sinpsi2; \
    afe_q3 =  cosphi2 * costheta2 * sinpsi2 - sinphi2 * sintheta2 * cospsi2; \
  }

/*
 * normalize quaternion
 */
#define AFE_NORM_QUAT() {			\
    FLOAT_T  mag = afe_q0*afe_q0 + afe_q1*afe_q1	\
      + afe_q2*afe_q2 + afe_q3*afe_q3;		\
    mag = sqrt( mag );				\
    afe_q0 /= mag;				\
    afe_q1 /= mag;				\
    afe_q2 /= mag;				\
    afe_q3 /= mag;				\
  }




/*
 * Compute the Jacobian of the measurements to the system states.
 */
#define AFE_COMPUTE_H_PHI() {          						\
    const FLOAT_T phi_err =  2 / (afe_dcm22*afe_dcm22 + afe_dcm12*afe_dcm12);     \
    afe_H[0] = (afe_q1 * afe_dcm22) * phi_err;				        \
    afe_H[1] = (afe_q0 * afe_dcm22 + 2 * afe_q1 * afe_dcm12) * phi_err;	        \
    afe_H[2] = (afe_q3 * afe_dcm22 + 2 * afe_q2 * afe_dcm12) * phi_err;	        \
    afe_H[3] = (afe_q2 * afe_dcm22) * phi_err;				        \
}

#define AFE_COMPUTE_H_THETA() {						        \
    const FLOAT_T theta_err  = -2 / sqrt( 1 - afe_dcm02*afe_dcm02 );	        \
    afe_H[0] = -afe_q2 * theta_err;						\
    afe_H[1] =  afe_q3 * theta_err;						\
    afe_H[2] = -afe_q0 * theta_err;						\
    afe_H[3] =  afe_q1 * theta_err;						\
  }

#define AFE_COMPUTE_H_PSI() {					                \
  const FLOAT_T psi_err =  2 / (afe_dcm00*afe_dcm00 + afe_dcm01*afe_dcm01);       \
  afe_H[0] = (afe_q3 * afe_dcm00) * psi_err;					\
  afe_H[1] = (afe_q2 * afe_dcm00) * psi_err;					\
  afe_H[2] = (afe_q1 * afe_dcm00 + 2 * afe_q2 * afe_dcm01) * psi_err;	        \
  afe_H[3] = (afe_q0 * afe_dcm00 + 2 * afe_q3 * afe_dcm01) * psi_err;	        \
  }


/* convert sensor reading into euler angle measurement */
static inline FLOAT_T afe_phi_of_accel( const FLOAT_T* accel ) {
  return atan2(accel[AXIS_Y], accel[AXIS_Z]);
}

static inline FLOAT_T afe_theta_of_accel( const FLOAT_T* accel) {
  FLOAT_T g2 =
    accel[AXIS_X]*accel[AXIS_X] +
    accel[AXIS_Y]*accel[AXIS_Y] +
    accel[AXIS_Z]*accel[AXIS_Z];
  return -asin( accel[AXIS_X] / sqrt( g2 ) );
}
/*
 * The rotation matrix to rotate from NED frame to body frame without
 * rotating in the yaw axis is:
 *
 * [ 1      0         0    ] [ cos(Theta)  0  -sin(Theta) ]
 * [ 0  cos(Phi)  sin(Phi) ] [      0      1       0      ]
 * [ 0 -sin(Phi)  cos(Phi) ] [ sin(Theta)  0   cos(Theta) ]
 *
 * This expands to:
 *
 * [  cos(Theta)              0      -sin(Theta)         ]
 * [  sin(Phi)*sin(Theta)  cos(Phi)   sin(Phi)*cos(Theta)]
 * [  cos(Phi)*sin(Theta)  -sin(Phi)  cos(Phi)*cos(Theta)]
 *
 * However, to untilt the compass reading, we need to use the
 * transpose of this matrix.
 *
 * [  cos(Theta)  sin(Phi)*sin(Theta)  cos(Phi)*sin(Theta) ]
 * [      0       cos(Phi)            -sin(Phi)            ]
 * [ -sin(Theta)  sin(Phi)*cos(Theta)  cos(Phi)*cos(Theta) ]
 *
 * Additionally,
 * since we already have the DCM computed for our current attitude,
 * we can short cut all of the trig. substituting
 * in from the definition of euler2quat and quat2euler, we have:
 *
 * [ cos(Theta)   -dcm12*dcm02   -dcm22*dcm02 ]
 * [    0          dcm22         -dcm12       ]
 * [ dcm02         dcm12          dcm22       ]
 *
 */
static inline FLOAT_T afe_psi_of_mag( const int16_t* mag ) {
  const FLOAT_T ctheta  = cos( afe_theta );
#if 0
  const FLOAT_T    mn = ctheta * mag[0]
    - (afe_dcm12 * mag[1] + afe_dcm22 * mag[2]) * afe_dcm02 / ctheta;

  const FLOAT_T    me =
    (afe_dcm22 * mag[1] - afe_dcm12 * mag[2]) / ctheta;
#else
  const FLOAT_T stheta  = sin( afe_theta );
  const FLOAT_T cphi  = cos( afe_phi );
  const FLOAT_T sphi  = sin( afe_phi );

  const FLOAT_T mn =
    ctheta*      mag[0]+
    sphi*stheta* mag[1]+
    cphi*stheta* mag[2];
  const FLOAT_T me =
    0*     mag[0]+
    cphi*  mag[1]+
    -sphi* mag[2];
#endif

  const FLOAT_T psi = -atan2( me, mn );
  return psi;
}

#define AFE_WARP(x, b) {			\
    while( x < -b )				\
      x += 2 * b;				\
    while( x > b )				\
      x -= 2 * b;				\
  }



#endif /* AHRS_QUAT_FAST_EKF_UTILS_H */
