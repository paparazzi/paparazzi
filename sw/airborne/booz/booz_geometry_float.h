#ifndef BOOZ_GEOMETRY_FLOAT_H
#define BOOZ_GEOMETRY_FLOAT_H

#include <math.h>

#ifndef RadOfDeg
#define RadOfDeg(d) ( (d)*M_PI/180. )
#define DegOfRad(r) ( (r)/M_PI*180. )
#endif

struct booz_fquat {
  FLOAT_T qi;
  FLOAT_T qx;
  FLOAT_T qy;
  FLOAT_T qz;
};

struct booz_fvect {
  FLOAT_T x;
  FLOAT_T y;
  FLOAT_T z;
};

struct booz_feuler {
  FLOAT_T phi;
  FLOAT_T theta;
  FLOAT_T psi;
};


#define BOOZ_FQUAT_ZERO(q) {						\
    q.qi = 1.;								\
    q.qx = 0.;								\
    q.qy = 0.;								\
    q.qz = 0.;								\
  }


#define BOOZ_FQUAT_COPY(_qo, _qi) {					\
    _qo.qi = _qi.qi;							\
    _qo.qx = _qi.qx;							\
    _qo.qy = _qi.qy;							\
    _qo.qz = _qi.qz;							\
  }


#define BOOZ_FQUAT_INVERT(_qo, _qi) {					\
    _qo.qi = _qi.qi;							\
    _qo.qx = -_qi.qx;							\
    _qo.qy = -_qi.qy;							\
    _qo.qz = -_qi.qz;							\
  }


#define BOOZ_FQUAT_MULT(c, a, b) {					\
    c.qi = a.qi*b.qi - a.qx*b.qx - a.qy*b.qy - a.qz*b.qz;		\
    c.qx = a.qi*b.qx + a.qx*b.qi + a.qy*b.qz - a.qz*b.qy;		\
    c.qy = a.qi*b.qy - a.qx*b.qz + a.qy*b.qi + a.qz*b.qx;		\
    c.qz = a.qi*b.qz + a.qx*b.qy - a.qy*b.qx + a.qz*b.qi;		\
  }


#define BOOZ_FQUAT_DIV(b, a, c) {					\
    b.qi = c.qi*a.qi + c.qx*a.qx + c.qy*a.qy + c.qz*a.qz;		\
    b.qx = c.qx*a.qi - c.qi*a.qx - c.qz*a.qy + c.qy*a.qz;		\
    b.qy = c.qy*a.qi + c.qz*a.qx - c.qi*a.qy - c.qx*a.qz;		\
    b.qz = c.qz*a.qi - c.qy*a.qx + c.qx*a.qy - c.qi*a.qz;		\
  }



#define BOOZ_FQUAT_NORM(n, q) {                                         \
    n = sqrt(q.qi*q.qi+q.qx*q.qx+q.qy*q.qy+q.qz*q.qz);			\
  }


#define BOOZ_FQUAT_NORMALISE(q) {		                        \
    FLOAT_T norm;							\
    BOOZ_FQUAT_NORM(norm, q);						\
    q.qi = q.qi / norm;							\
    q.qx = q.qx / norm;							\
    q.qy = q.qy / norm;							\
    q.qz = q.qz / norm;							\
  }


#define BOOZ_FQUAT_VMULT(v_out, q, v_in) {				\
    const FLOAT_T qi2  = q.qi*q.qi;					\
    const FLOAT_T qiqx = q.qi*q.qx;					\
    const FLOAT_T qiqy = q.qi*q.qy;					\
    const FLOAT_T qiqz = q.qi*q.qz;					\
    const FLOAT_T qx2  = q.qx*q.qx;					\
    const FLOAT_T qxqy = q.qx*q.qy;					\
    const FLOAT_T qxqz = q.qx*q.qz;					\
    const FLOAT_T qy2  = q.qy*q.qy;					\
    const FLOAT_T qyqz = q.qy*q.qz;					\
    const FLOAT_T qz2 = q.qz*q.qz;					\
    const FLOAT_T m00 = qi2 + qx2 - qy2 - qz2;				\
    const FLOAT_T m01 = 2 * ( qxqy + qiqz );				\
    const FLOAT_T m02 = 2 * ( qxqz - qiqy );				\
    const FLOAT_T m10 = 2 * ( qxqy - qiqz );				\
    const FLOAT_T m11 = qi2 - qx2 + qy2 - qz2;				\
    const FLOAT_T m12 = 2 * ( qyqz + qiqx );				\
    const FLOAT_T m20 = 2 * ( qxqz + qiqy );				\
    const FLOAT_T m21 = 2 * ( qyqz - qiqx );				\
    const FLOAT_T m22 = qi2 - qx2 - qy2 + qz2;				\
    v_out.x = m00 * v_in.x + m01 * v_in.y + m02 * v_in.z;		\
    v_out.y = m10 * v_in.x + m11 * v_in.y + m12 * v_in.z;		\
    v_out.z = m20 * v_in.x + m21 * v_in.y + m22 * v_in.z;		\
  }



#define BOOZ_FQUAT_OF_EULER(q, e) {                                     \
    const FLOAT_T phi2   = e.phi / 2.0;					\
    const FLOAT_T theta2 = e.theta / 2.0;				\
    const FLOAT_T psi2   = e.psi / 2.0;					\
									\
    const FLOAT_T sinphi2 = sin( phi2 );				\
    const FLOAT_T cosphi2 = cos( phi2 );				\
    const FLOAT_T sintheta2 = sin( theta2 );				\
    const FLOAT_T costheta2 = cos( theta2 );				\
    const FLOAT_T sinpsi2   = sin( psi2 );				\
    const FLOAT_T cospsi2   = cos( psi2 );				\
									\
    q.qi =  cosphi2 * costheta2 * cospsi2 + sinphi2 * sintheta2 * sinpsi2; \
    q.qx = -cosphi2 * sintheta2 * sinpsi2 + sinphi2 * costheta2 * cospsi2; \
    q.qy =  cosphi2 * sintheta2 * cospsi2 + sinphi2 * costheta2 * sinpsi2; \
    q.qz =  cosphi2 * costheta2 * sinpsi2 - sinphi2 * sintheta2 * cospsi2; \
  }


#define BOOZ_FEULER_OF_QUAT(e,q) {					\
    									\
    const FLOAT_T qx2  = q.qx*q.qx;					\
    const FLOAT_T qy2  = q.qy*q.qy;					\
    const FLOAT_T qz2  = q.qz*q.qz;					\
    const FLOAT_T qiqx = q.qi*q.qx;					\
    const FLOAT_T qiqy = q.qi*q.qy;					\
    const FLOAT_T qiqz = q.qi*q.qz;					\
    const FLOAT_T qxqy = q.qx*q.qy;					\
    const FLOAT_T qxqz = q.qx*q.qz;					\
    const FLOAT_T qyqz = q.qy*q.qz;					\
    const FLOAT_T dcm00 = 1.0 - 2.*(  qy2 +  qz2 );			\
    const FLOAT_T dcm01 =       2.*( qxqy + qiqz );			\
    const FLOAT_T dcm02 =       2.*( qxqz - qiqy );			\
    const FLOAT_T dcm12 =       2.*( qyqz + qiqx );			\
    const FLOAT_T dcm22 = 1.0 - 2.*(  qx2 +  qy2 );			\
									\
    e.phi   =  atan2( dcm12, dcm22 );					\
    e.theta = -asin( dcm02 );						\
    e.psi   = atan2( dcm01, dcm00 );					\
									\
  }


#define BOOZ_FEULER_OF_QUAT_ALT(e,q) {					\
    const FLOAT_T qi2 = q.qi*q.qi;					\
    const FLOAT_T qx2 = q.qx*q.qx;					\
    const FLOAT_T qy2 = q.qy*q.qy;					\
    const FLOAT_T qz2 = q.qz*q.qz;					\
    const FLOAT_T qiqx = q.qi*q.qx;					\
    const FLOAT_T qiqy = q.qi*q.qy;					\
    const FLOAT_T qiqz = q.qi*q.qz;					\
    const FLOAT_T qxqy = q.qx*q.qy;					\
    const FLOAT_T qxqz = q.qx*q.qz;					\
    const FLOAT_T qyqz = q.qy*q.qz;					\
    e.phi   =  atan2( 2.*(qiqx + qyqz), qi2-qx2-qy2+qz2 );		\
    e.theta =  asin( 2.*(qiqy - qxqz ));				\
    e.psi   =  atan2( 2.*(qiqz + qxqy), qi2+qx2-qy2-qz2 );		\
  }


#define BOOZ_FVECT_ASSIGN(v, _x, _y, _z) {				\
    v.x = _x;								\
    v.y = _y;								\
    v.z = _z;								\
  }


#define BOOZ_FVECT_ZERO(v) {						\
    BOOZ_FVECT_ASSIGN(v, 0., 0., 0.);					\
  }


#define BOOZ_FVECT_NORM(n, v) {			                        \
    const FLOAT_T n2 = v.x*v.x + v.y*v.y + v.z*v.z;			\
    n = sqrt(n2);							\
  }

#define BOOZ_FVECT_CROSS_PRODUCT(vo, v1, v2) {				\
    vo.x = v1.y*v2.z - v1.z*v2.y;					\
    vo.y = v1.z*v2.x - v1.x*v2.z;					\
    vo.z = v1.x*v2.y - v1.y*v2.x;					\
  }


#define BOOZ_FVECT_SMUL(vo, vi, s) {                                    \
    vo.x = vi.x * s;							\
    vo.y = vi.y * s;							\
    vo.z = vi.z * s;							\
  }


#define BOOZ_FEULER_ASSIGN(e, _phi, _theta, _psi) {			\
    e.phi = _phi;							\
    e.theta = _theta;							\
    e.psi = _psi;							\
  }


#define BOOZ_FEULER_ASSIGN_DEG(e, _phi, _theta, _psi) {			\
    BOOZ_FEULER_ASSIGN(e, RadOfDeg(_phi), RadOfDeg(_theta), RadOfDeg(_psi)); \
  }


#define BOOZ_FEULER_ZERO(e) {                                           \
    BOOZ_FEULER_ASSIGN(e, 0., 0., 0.);					\
  }


#endif /* BOOZ_GEOMETRY_FLOAT_H */

