#ifndef AHRS_UTILS_H
#define AHRS_UTILS_H

#include "frames.h"

#ifdef DEBUG
#include <stdio.h>

#define PrintEuler() {					\
    printf("euler %f %f %f\n\n", phi, theta, psi);	\
  }

#define PrintQuat() {						\
    float norm = sqrt ( q0*q0 + q1*q1 + q2*q2 + q3*q3);		\
    printf("quat %f %f %f %f (%f)\n\n", q0, q1, q2, q3, norm);	\
  }

#define PrintDCM() {					\
    printf("DCM %.2f %.2f %.2f\n", dcm00, dcm01, dcm02);	\
    printf("    XXXX XXXX %.2f\n", dcm12);			\
    printf("    XXXX XXXX %.2f\n\n", dcm22);			\
  }

#define PrintC() {					\
    printf("C %.2f %.2f %.2f %.2f\n\n", C[0], C[1], C[2], C[3]); \
  }
#endif /* DEBUG */

extern float C[4];

extern float dcm00;
extern float dcm01;
extern float dcm02;
extern float dcm12;
extern float dcm22;


/*
 * Compute the five elements of the DCM that we use for our
 * rotations and Jacobians.  This is used by several other functions
 * to speedup their computations.
 */
static inline void DCM_of_quat( void ) {
  dcm00 = 1.0-2*(q2*q2 + q3*q3);
  dcm01 =     2*(q1*q2 + q0*q3);
  dcm02 =     2*(q1*q3 - q0*q2);
  dcm12 =     2*(q2*q3 + q0*q1);
  dcm22 = 1.0-2*(q1*q1 + q2*q2);
}

static inline void eulers_of_DCM ( void ) {
 ahrs_phi = atan2( dcm12, dcm22 );
 ahrs_theta = -asin( dcm02 );
 ahrs_psi = atan2( dcm01, dcm00 );
}


/*
 * initialise a quaternion from a set of eulers
 */
static inline void quat_of_eulers ( void ) {
  const float phi2     = ahrs_phi / 2.0;
  const float theta2   = ahrs_theta / 2.0;
  const float psi2     = ahrs_psi / 2.0;

  const float sinphi2   = sin( phi2 );
  const float cosphi2   = cos( phi2 );

  const float sintheta2 = sin( theta2 );
  const float costheta2 = cos( theta2 );

  const float sinpsi2   = sin( psi2 );
  const float cospsi2   = cos( psi2 );

  q0 =  cosphi2 * costheta2 * cospsi2 + sinphi2 * sintheta2 * sinpsi2;
  q1 = -cosphi2 * sintheta2 * sinpsi2 + sinphi2 * costheta2 * cospsi2;
  q2 =  cosphi2 * sintheta2 * cospsi2 + sinphi2 * costheta2 * sinpsi2;
  q3 =  cosphi2 * costheta2 * sinpsi2 - sinphi2 * sintheta2 * cospsi2;
}

/*
 * normalize quaternion
 */
static inline void norm_quat( void ) {
  float  mag = q0*q0 + q1*q1 + q2*q2 + q3*q3;
  mag = sqrt( mag );
  q0 /= mag;
  q1 /= mag;
  q2 /= mag;
  q3 /= mag;
}

/*
 * Compute the Jacobian of the measurements to the system states.
 * You must have already computed the DCM for the quaternion before
 * calling this function.
 */
#if 1
static inline void compute_dphi_dq( void ) {
  const float phi_err =  2 / (dcm22*dcm22 + dcm12*dcm12);

  C[0] = (q1 * dcm22) * phi_err;
  C[1] = (q0 * dcm22 + 2 * q1 * dcm12) * phi_err;
  C[2] = (q3 * dcm22 + 2 * q2 * dcm12) * phi_err;
  C[3] = (q2 * dcm22) * phi_err;
}

static inline void compute_dtheta_dq( void ) {
  const float theta_err  = -2 / sqrt( 1 - dcm02*dcm02 );

  C[0] = -q2 * theta_err;
  C[1] =  q3 * theta_err;
  C[2] = -q0 * theta_err;
  C[3] =  q1 * theta_err;
}

static inline void compute_dpsi_dq( void ) {
  const float psi_err    =  2 / (dcm00*dcm00 + dcm01*dcm01);

  C[0] = (q3 * dcm00) * psi_err;
  C[1] = (q2 * dcm00) * psi_err;
  C[2] = (q1 * dcm00 + 2 * q2 * dcm01) * psi_err;
  C[3] = (q0 * dcm00 + 2 * q3 * dcm01) * psi_err;
}
#else
/* paper JSchlep p85 */
static inline void compute_dphi_dq( void ) {
  const float a = (q2+q1)*(q2+q1) + (q3+q0)*(q3+q0);
  const float b = (q2-q1)*(q2-q1) + (q3-q0)*(q3-q0);

  C[0] = -(q2+q1)/a - (q2-q1)/b;
  C[1] =  (q3+q0)/a + (q3-q0)/b;
  C[2] =  (q3+q0)/a - (q3-q0)/b;
  C[3] = -(q2+q1)/a + (q2-q1)/b; 
}

static inline void compute_dtheta_dq( void ) {
  const float a  = 2 / sqrt( 1 - 4*(q1*q2 + q0*q3)*(q1*q2 + q0*q3));
  
  C[0] = q3 * a;
  C[1] = q2 * a;
  C[2] = q1 * a;
  C[3] = q0 * a;
}

static inline void compute_dpsi_dq( void ) {
  const float a = (q2+q1)*(q2+q1) + (q3+q0)*(q3+q0);
  const float b = (q2-q1)*(q2-q1) + (q3-q0)*(q3-q0);

  C[0] =  -(q2+q1)/a + (q2-q1)/b;
  C[1] =   (q3+q0)/a - (q3-q0)/b;
  C[2] =   (q3+q0)/a + (q3-q0)/b;
  C[3] =  -(q2+q1)/a - (q2-q1)/b; 
}
#endif

static inline float ahrs_roll_of_accel( const float* accel ) {
  return atan2(accel[AXIS_Y], accel[AXIS_Z]);
}

static inline float ahrs_pitch_of_accel( const float* accel) {
  float g2 =					
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
static inline float ahrs_yaw_of_mag( const int16_t* mag ) {
  const float ctheta  = cos( ahrs_theta );
#if 1
  const float    mn = ctheta * mag[0] 
    - (dcm12 * mag[1] + dcm22 * mag[2]) * dcm02 / ctheta;
  
  const float    me =
    (dcm22 * mag[1] - dcm12 * mag[2]) / ctheta;
#else
  const float stheta  = sin( ahrs_theta );
  const float cphi  = cos( ahrs_phi );
  const float sphi  = sin( ahrs_phi );

  const float mn = 
    ctheta*      mag[0]+
    sphi*stheta* mag[1]+
    cphi*stheta* mag[2];
  const float me = 
    0*     mag[0]+
    cphi*  mag[1]+
    -sphi* mag[2];
#endif
  
  const float yaw = -atan2( me, mn );
  return yaw;
}





#endif /* AHRS_UTILS_H */
