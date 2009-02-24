#ifndef PPRZ_ALGEBRA_FLOAT_H
#define PPRZ_ALGEBRA_FLOAT_H

#include "pprz_algebra.h"


struct FloatVect2 {
  FLOAT_T x;
  FLOAT_T y;
};

struct FloatVect3 {
  FLOAT_T x;
  FLOAT_T y;
  FLOAT_T z;
};

struct FloatQuat {
  FLOAT_T qi;
  FLOAT_T qx;
  FLOAT_T qy;
  FLOAT_T qz;
};

struct FloatMat33 {
  FLOAT_T m[3*3];
};

struct FloatEulers {
  FLOAT_T phi;
  FLOAT_T theta;
  FLOAT_T psi;
};

struct FloatRates {
  FLOAT_T p;
  FLOAT_T q;
  FLOAT_T r;
};


#define FLOAT_VECT3_ZERO(_v) VECT3_ASSIGN(_v, 0., 0., 0.)	\
    
#define FLOAT_VECT3_DIFF(_c, _a, _b) {		\
    (_c).x = (_a).x - (_b).x;			\
    (_c).y = (_a).y - (_b).y;			\
    (_c).z = (_a).z - (_b).z;			\
  }

#define FLOAT_VECT3_SMUL(_vo, _s, _vi) {		\
    (_vo).x = (_s) * (_vi).x;				\
    (_vo).y = (_s) * (_vi).y;				\
    (_vo).z = (_s) * (_vi).z;				\
  }

#define FLOAT_VECT3_NORM(n, v) {					\
    const float n2 = v.x*v.x + v.y*v.y + v.z*v.z;			\
    n = sqrtf(n2);							\
  }

#define FLOAT_VECT3_CROSS_PRODUCT(vo, v1, v2) {				\
    vo.x = v1.y*v2.z - v1.z*v2.y;					\
    vo.y = v1.z*v2.x - v1.x*v2.z;					\
    vo.z = v1.x*v2.y - v1.y*v2.x;					\
  }


/*
 * Quaternions
 */

#define FLOAT_QUAT_ZERO(_q) {						\
    (_q).qi = 1.;							\
    (_q).qx = 0.;							\
    (_q).qy = 0.;							\
    (_q).qz = 0.;							\
  }

#define FLOAT_QUAT_COPY(_qo, _qi) {					\
    (_qo).qi = (_qi).qi;						\
    (_qo).qx = (_qi).qx;						\
    (_qo).qy = (_qi).qy;						\
    (_qo).qz = (_qi).qz;						\
  }

#define FLOAT_QUAT_NORM(_n, _q) {					\
    _n = sqrtf(_q.qi*_q.qi+_q.qx*_q.qx+_q.qy*_q.qy+_q.qz*_q.qz);	\
  }

#define FLOAT_QUAT_NORMALISE(q) {		                        \
    float norm;								\
    FLOAT_QUAT_NORM(norm, q);						\
    q.qi = q.qi / norm;							\
    q.qx = q.qx / norm;							\
    q.qy = q.qy / norm;							\
    q.qz = q.qz / norm;							\
  }

#define FLOAT_QUAT_INVERT(_qo, _qi) {					\
    (_qo).qi =  (_qi).qi;						\
    (_qo).qx = -(_qi).qx;						\
    (_qo).qy = -(_qi).qy;						\
    (_qo).qz = -(_qi).qz;						\
  }

#define FLOAT_QUAT_MULT(_c, _a, _b) {					             \
    (_c).qi = (_a).qi*(_b).qi - (_a).qx*(_b).qx - (_a).qy*(_b).qy - (_a).qz*(_b).qz; \
    (_c).qx = (_a).qi*(_b).qx + (_a).qx*(_b).qi + (_a).qy*(_b).qz - (_a).qz*(_b).qy; \
    (_c).qy = (_a).qi*(_b).qy - (_a).qx*(_b).qz + (_a).qy*(_b).qi + (_a).qz*(_b).qx; \
    (_c).qz = (_a).qi*(_b).qz + (_a).qx*(_b).qy - (_a).qy*(_b).qx + (_a).qz*b.qi;    \
  }

#define FLOAT_QUAT_DIV(b, a, c) {					\
    b.qi = c.qi*a.qi + c.qx*a.qx + c.qy*a.qy + c.qz*a.qz;		\
    b.qx = c.qx*a.qi - c.qi*a.qx - c.qz*a.qy + c.qy*a.qz;		\
    b.qy = c.qy*a.qi + c.qz*a.qx - c.qi*a.qy - c.qx*a.qz;		\
    b.qz = c.qz*a.qi - c.qy*a.qx + c.qx*a.qy - c.qi*a.qz;		\
  }

#define FLOAT_QUAT_VMULT(v_out, q, v_in) {				\
    const float qi2  = q.qi*q.qi;					\
    const float qiqx = q.qi*q.qx;					\
    const float qiqy = q.qi*q.qy;					\
    const float qiqz = q.qi*q.qz;					\
    const float qx2  = q.qx*q.qx;					\
    const float qxqy = q.qx*q.qy;					\
    const float qxqz = q.qx*q.qz;					\
    const float qy2  = q.qy*q.qy;					\
    const float qyqz = q.qy*q.qz;					\
    const float qz2 = q.qz*q.qz;					\
    const float m00 = qi2 + qx2 - qy2 - qz2;				\
    const float m01 = 2 * ( qxqy + qiqz );				\
    const float m02 = 2 * ( qxqz - qiqy );				\
    const float m10 = 2 * ( qxqy - qiqz );				\
    const float m11 = qi2 - qx2 + qy2 - qz2;				\
    const float m12 = 2 * ( qyqz + qiqx );				\
    const float m20 = 2 * ( qxqz + qiqy );				\
    const float m21 = 2 * ( qyqz - qiqx );				\
    const float m22 = qi2 - qx2 - qy2 + qz2;				\
    v_out.x = m00 * v_in.x + m01 * v_in.y + m02 * v_in.z;		\
    v_out.y = m10 * v_in.x + m11 * v_in.y + m12 * v_in.z;		\
    v_out.z = m20 * v_in.x + m21 * v_in.y + m22 * v_in.z;		\
  }

#define FLOAT_QUAT_OF_EULERS(_q, _e) {					\
    									\
    const float phi2   = (_e).phi/ 2.0;					\
    const float theta2 = (_e).theta/2.0;				\
    const float psi2   = (_e).psi/2.0;					\
									\
    const float s_phi2   = sinf( phi2 );				\
    const float c_phi2   = cosf( phi2 );				\
    const float s_theta2 = sinf( theta2 );				\
    const float c_theta2 = cosf( theta2 );				\
    const float s_psi2   = sinf( psi2 );				\
    const float c_psi2   = cosf( psi2 );				\
									\
    (_q).qi =  c_phi2 * c_theta2 * c_psi2 + s_phi2 * s_theta2 * s_psi2; \
    (_q).qx = -c_phi2 * s_theta2 * s_psi2 + s_phi2 * c_theta2 * c_psi2; \
    (_q).qy =  c_phi2 * s_theta2 * c_psi2 + s_phi2 * c_theta2 * s_psi2; \
    (_q).qz =  c_phi2 * c_theta2 * s_psi2 - s_phi2 * s_theta2 * c_psi2; \
    									\
  }


/*
 *  Euler angles
 */

#define FLOAT_EULERS_ZERO(_e) EULERS_ASSIGN(_e, 0., 0., 0.);

#define FLOAT_EULERS_OF_QUAT(_e, _q) {					\
									\
    const float qx2  = q.qx*q.qx;					\
    const float qy2  = q.qy*q.qy;					\
    const float qz2  = q.qz*q.qz;					\
    const float qiqx = q.qi*q.qx;					\
    const float qiqy = q.qi*q.qy;					\
    const float qiqz = q.qi*q.qz;					\
    const float qxqy = q.qx*q.qy;					\
    const float qxqz = q.qx*q.qz;					\
    const float qyqz = q.qy*q.qz;					\
    const float dcm00 = 1.0 - 2.*(  qy2 +  qz2 );			\
    const float dcm01 =       2.*( qxqy + qiqz );			\
    const float dcm02 =       2.*( qxqz - qiqy );			\
    const float dcm12 =       2.*( qyqz + qiqx );			\
    const float dcm22 = 1.0 - 2.*(  qx2 +  qy2 );			\
									\
    (_e).phi = atan2f( dcm12, dcm22 );					\
    (_e).theta = -asinf( dcm02 );					\
    (_e).psi = atan2f( dcm01, dcm00 );					\
									\
  }





#endif /* PPRZ_ALGEBRA_FLOAT_H */
