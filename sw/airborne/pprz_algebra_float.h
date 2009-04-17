#ifndef PPRZ_ALGEBRA_FLOAT_H
#define PPRZ_ALGEBRA_FLOAT_H

#include "pprz_algebra.h"

#include <math.h>

struct FloatVect2 {
  float x;
  float y;
};

struct FloatVect3 {
  float x;
  float y;
  float z;
};

struct FloatQuat {
  float qi;
  float qx;
  float qy;
  float qz;
};

struct FloatMat33 {
  float m[3*3];
};

struct FloatRMat {
  float m[3*3];
};

struct FloatEulers {
  float phi;
  float theta;
  float psi;
};

struct FloatRates {
  float p;
  float q;
  float r;
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

#define FLOAT_VECT3_NORM(_v) (sqrtf((_v).x*(_v).x + (_v).y*(_v).y + (_v).z*(_v).z))

#define FLOAT_VECT3_CROSS_PRODUCT(vo, v1, v2) {				\
    vo.x = v1.y*v2.z - v1.z*v2.y;					\
    vo.y = v1.z*v2.x - v1.x*v2.z;					\
    vo.z = v1.x*v2.y - v1.y*v2.x;					\
  }

/*
 * Rotation Matrices
 */

/* */
#define FLOAT_RMAT_ZERO(_rm) {			\
    (_rm).m[0] = 1.;                            \
    (_rm).m[1] = 0.;                            \
    (_rm).m[2] = 0.;                            \
    (_rm).m[3] = 0.;                            \
    (_rm).m[4] = 1.;                            \
    (_rm).m[5] = 0.;                            \
    (_rm).m[6] = 0.;                            \
    (_rm).m[7] = 0.;                            \
    (_rm).m[8] = 1.;                            \
  }


/* _m_a2c = _m_a2b comp _m_b2c , aka  _m_a2c = _m_b2c * _m_a2b */
#define FLOAT_RMAT_COMP(_m_a2c, _m_a2b, _m_b2c) {			\
    _m_a2c.m[0] = (_m_b2c.m[0]*_m_a2b.m[0] + _m_b2c.m[1]*_m_a2b.m[3] + _m_b2c.m[2]*_m_a2b.m[6]); \
    _m_a2c.m[1] = (_m_b2c.m[0]*_m_a2b.m[1] + _m_b2c.m[1]*_m_a2b.m[4] + _m_b2c.m[2]*_m_a2b.m[7]); \
    _m_a2c.m[2] = (_m_b2c.m[0]*_m_a2b.m[2] + _m_b2c.m[1]*_m_a2b.m[5] + _m_b2c.m[2]*_m_a2b.m[8]); \
    _m_a2c.m[3] = (_m_b2c.m[3]*_m_a2b.m[0] + _m_b2c.m[4]*_m_a2b.m[3] + _m_b2c.m[5]*_m_a2b.m[6]); \
    _m_a2c.m[4] = (_m_b2c.m[3]*_m_a2b.m[1] + _m_b2c.m[4]*_m_a2b.m[4] + _m_b2c.m[5]*_m_a2b.m[7]); \
    _m_a2c.m[5] = (_m_b2c.m[3]*_m_a2b.m[2] + _m_b2c.m[4]*_m_a2b.m[5] + _m_b2c.m[5]*_m_a2b.m[8]); \
    _m_a2c.m[6] = (_m_b2c.m[6]*_m_a2b.m[0] + _m_b2c.m[7]*_m_a2b.m[3] + _m_b2c.m[8]*_m_a2b.m[6]); \
    _m_a2c.m[7] = (_m_b2c.m[6]*_m_a2b.m[1] + _m_b2c.m[7]*_m_a2b.m[4] + _m_b2c.m[8]*_m_a2b.m[7]); \
    _m_a2c.m[8] = (_m_b2c.m[6]*_m_a2b.m[2] + _m_b2c.m[7]*_m_a2b.m[5] + _m_b2c.m[8]*_m_a2b.m[8]); \
  }

/* _m_a2b = _m_a2c comp_inv _m_b2c , aka  _m_a2b = inv(_m_b2c) * _m_a2c */
#define FLOAT_RMAT_COMP_INV(_m_a2b, _m_a2c, _m_b2c) {			\
    _m_a2b.m[0] = (_m_b2c.m[0]*_m_a2c.m[0] + _m_b2c.m[3]*_m_a2c.m[3] + _m_b2c.m[6]*_m_a2c.m[6]); \
    _m_a2b.m[1] = (_m_b2c.m[0]*_m_a2c.m[1] + _m_b2c.m[3]*_m_a2c.m[4] + _m_b2c.m[6]*_m_a2c.m[7]); \
    _m_a2b.m[2] = (_m_b2c.m[0]*_m_a2c.m[2] + _m_b2c.m[3]*_m_a2c.m[5] + _m_b2c.m[6]*_m_a2c.m[8]); \
    _m_a2b.m[3] = (_m_b2c.m[1]*_m_a2c.m[0] + _m_b2c.m[4]*_m_a2c.m[3] + _m_b2c.m[7]*_m_a2c.m[6]); \
    _m_a2b.m[4] = (_m_b2c.m[1]*_m_a2c.m[1] + _m_b2c.m[4]*_m_a2c.m[4] + _m_b2c.m[7]*_m_a2c.m[7]); \
    _m_a2b.m[5] = (_m_b2c.m[1]*_m_a2c.m[2] + _m_b2c.m[4]*_m_a2c.m[5] + _m_b2c.m[7]*_m_a2c.m[8]); \
    _m_a2b.m[6] = (_m_b2c.m[2]*_m_a2c.m[0] + _m_b2c.m[5]*_m_a2c.m[3] + _m_b2c.m[8]*_m_a2c.m[6]); \
    _m_a2b.m[7] = (_m_b2c.m[2]*_m_a2c.m[1] + _m_b2c.m[5]*_m_a2c.m[4] + _m_b2c.m[8]*_m_a2c.m[7]); \
    _m_a2b.m[8] = (_m_b2c.m[2]*_m_a2c.m[2] + _m_b2c.m[5]*_m_a2c.m[5] + _m_b2c.m[8]*_m_a2c.m[8]); \
  }


#define FLOAT_RMAT_NORM(_m) (						\
    sqrtf(SQUARE((_m).m[0])+ SQUARE((_m).m[1])+ SQUARE((_m).m[2])+	\
	  SQUARE((_m).m[3])+ SQUARE((_m).m[4])+ SQUARE((_m).m[5])+	\
	  SQUARE((_m).m[6])+ SQUARE((_m).m[7])+ SQUARE((_m).m[8]))	\
    )

#define FLOAT_RMAT_OF_EULERS(_rm, _e) {					\
    									\
    const float sphi   = sinf((_e).phi);				\
    const float cphi   = cosf((_e).phi);				\
    const float stheta = sinf((_e).theta);				\
    const float ctheta = cosf((_e).theta);				\
    const float spsi   = sinf((_e).psi);				\
    const float cpsi   = cosf((_e).psi);				\
    									\
    _rm.m[0] = ctheta*cpsi;						\
    _rm.m[1] = ctheta*spsi;						\
    _rm.m[2] = -stheta;							\
    _rm.m[3] = sphi*stheta*cpsi - cphi*spsi;				\
    _rm.m[4] = sphi*stheta*spsi + cphi*cpsi;				\
    _rm.m[5] = sphi*ctheta;						\
    _rm.m[6] = cphi*stheta*cpsi + sphi*spsi;				\
    _rm.m[7] = cphi*stheta*spsi - sphi*cpsi;				\
    _rm.m[8] = cphi*ctheta;						\
    									\
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

#define FLOAT_QUAT_NORM(_q) (sqrtf(SQUARE(_q.qi) + SQUARE(_q.qx)+	\
				   SQUARE(_q.qx) + SQUARE(_q.qy)))	\

#define FLOAT_QUAT_NORMALISE(q) {		                        \
    float norm = FLOAT_QUAT_NORM(q);					\
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

/* _a2c = _a2b comp _b2c , aka  _a2c = _b2c * _a2b */
#define FLOAT_QUAT_COMP(_a2c, _a2b, _b2c) {				\
    (_a2c).qi = (_a2b).qi*(_b2c).qi - (_a2b).qx*(_b2c).qx - (_a2b).qy*(_b2c).qy - (_a2b).qz*(_b2c).qz; \
    (_a2c).qx = (_a2b).qi*(_b2c).qx + (_a2b).qx*(_b2c).qi + (_a2b).qy*(_b2c).qz - (_a2b).qz*(_b2c).qy; \
    (_a2c).qy = (_a2b).qi*(_b2c).qy - (_a2b).qx*(_b2c).qz + (_a2b).qy*(_b2c).qi + (_a2b).qz*(_b2c).qx; \
    (_a2c).qz = (_a2b).qi*(_b2c).qz + (_a2b).qx*(_b2c).qy - (_a2b).qy*(_b2c).qx + (_a2b).qz*(_b2c).qi; \
  }

/* _a2b = _a2b comp_inv _b2c , aka  _a2b = inv(_b2c) * _a2c */
#define FLOAT_QUAT_COMP_INV(_a2b, _a2c, _b2c) {				\
    (_a2b).qi =  (_a2c).qi*(_b2c).qi + (_a2c).qx*(_b2c).qx + (_a2c).qy*(_b2c).qy + (_a2c).qz*(_b2c).qz; \
    (_a2b).qx = -(_a2c).qi*(_b2c).qx + (_a2c).qx*(_b2c).qi - (_a2c).qy*(_b2c).qz + (_a2c).qz*(_b2c).qy; \
    (_a2b).qy = -(_a2c).qi*(_b2c).qy + (_a2c).qx*(_b2c).qz + (_a2c).qy*(_b2c).qi - (_a2c).qz*(_b2c).qx; \
    (_a2b).qz = -(_a2c).qi*(_b2c).qz - (_a2c).qx*(_b2c).qy + (_a2c).qy*(_b2c).qx + (_a2c).qz*(_b2c).qi; \
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

#define FLOAT_EULERS_NORM(_e) (sqrtf(SQUARE((_e).phi)+SQUARE((_e).theta)+SQUARE((_e).psi))) ;

#define FLOAT_EULERS_OF_RMAT(_e, _rm) {					\
    									\
    const float dcm00 = (_rm).m[0];					\
    const float dcm01 = (_rm).m[1];					\
    const float dcm02 = (_rm).m[2];					\
    const float dcm12 = (_rm).m[5];					\
    const float dcm22 = (_rm).m[8];					\
    (_e).phi   = atan2f( dcm12, dcm22 );				\
    (_e).theta = -asinf( dcm02 );					\
    (_e).psi   = atan2f( dcm01, dcm00 );				\
									\
  }

#define FLOAT_EULERS_OF_QUAT(_e, _q) {					\
									\
    const float qx2  = (_q).qx*(_q).qx;					\
    const float qy2  = (_q).qy*(_q).qy;					\
    const float qz2  = (_q).qz*(_q).qz;					\
    const float qiqx = (_q).qi*(_q).qx;					\
    const float qiqy = (_q).qi*(_q).qy;					\
    const float qiqz = (_q).qi*(_q).qz;					\
    const float qxqy = (_q).qx*(_q).qy;					\
    const float qxqz = (_q).qx*(_q).qz;					\
    const float qyqz = (_q).qy*(_q).qz;					\
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
