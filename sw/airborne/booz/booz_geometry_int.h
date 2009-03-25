#ifndef BOOZ_GEOMETRY_INT_H
#define BOOZ_GEOMETRY_INT_H

#include <inttypes.h>


#define IPOS_OF_CM 2.56
#define IPOS_OF_CM_NUM 64
#define IPOS_OF_CM_DEN 25
#define ISPEED_OF_CM_S 5242.88
#define ISPEED_OF_CM_S_NUM 41943
#define ISPEED_OF_CM_S_DEN 8


#define IQUAT_RES  15
#define IPOS_FRAC   8
/* max 8 rad/s for 16bits */
#define IRATE_RES  12
#define IANGLE_RES 12
#define IACCEL_RES 10
#define ISPEED_RES 19
#define IMAG_RES   11
#define ITRIG_RES  14

#define PI_2_INT   (int32_t)(   1.5707963267948966192313216916397514*(1<<IANGLE_RES))
#define PI_INT     (int32_t)(   3.1415926535897932384626433832795029*(1<<IANGLE_RES))
#define TWO_PI_INT (int32_t)(2.*3.1415926535897932384626433832795029*(1<<IANGLE_RES))

#define BOOZ_ANGLE_NORMALIZE(_a) {		\
    while (_a > PI_INT)  _a -= TWO_PI_INT;	\
    while (_a < -PI_INT) _a += TWO_PI_INT;	\
  }

#define BOOZ_IMULT(_a, _b, _r) (((_a)*(_b))>>(_r))

struct booz_iquat {
  int32_t qi;
  int32_t qx;
  int32_t qy;
  int32_t qz;
};

struct booz_ivect2 {
  int32_t x;
  int32_t y;
};

struct booz_ivect {
  int32_t x;
  int32_t y;
  int32_t z;
};

struct booz_ieuler {
  int32_t phi;
  int32_t theta;
  int32_t psi;
};


struct Pprz_int16_vect2 {
  int16_t x;
  int16_t y;
};

struct Pprz_int16_vect3 {
  int16_t x;
  int16_t y;
  int16_t z;
};

struct Pprz_int16_rate {
  int16_t p;
  int16_t q;
  int16_t r;
};

struct Pprz_int16_euler {
  int16_t phi;
  int16_t theta;
  int16_t psi;
};

struct Pprz_int16_quat {
  int16_t qi;
  int16_t qx;
  int16_t qy;
  int16_t qz;
};

struct Pprz_int32_vect2 {
  int32_t x;
  int32_t y;
};

struct Pprz_int32_vect3 {
  int32_t x;
  int32_t y;
  int32_t z;
};

struct Pprz_int32_rate {
  int32_t p;
  int32_t q;
  int32_t r;
};

struct Pprz_int32_euler {
  int32_t phi;
  int32_t theta;
  int32_t psi;
};

struct Pprz_int32_lla {
  int32_t lat;
  int32_t lon;
  int32_t alt;
};

struct Pprz_int32_quat {
  int32_t qi;
  int32_t qx;
  int32_t qy;
  int32_t qz;
};


#define PPRZ_INT16_OF_INT32_VECT2(_o, _i) {	\
    _o.x = (int16_t)_i.x;			\
    _o.y = (int16_t)_i.y;			\
  }

#define PPRZ_INT16_OF_INT32_VECT3(_o, _i) {	\
    _o.x = (int16_t)_i.x;			\
    _o.y = (int16_t)_i.y;			\
    _o.z = (int16_t)_i.z;			\
  }

#define PPRZ_INT16_OF_INT32_RATE(_o, _i) {	\
    _o.p = (int16_t)_i.p;			\
    _o.q = (int16_t)_i.q;			\
    _o.r = (int16_t)_i.r;			\
  }

#define PPRZ_INT16_OF_INT32_EULER(_o, _i) {	\
    _o.phi   = (int16_t)_i.phi;			\
    _o.theta = (int16_t)_i.theta;		\
    _o.psi   = (int16_t)_i.psi;			\
  }

#define PPRZ_INT32_VECT3_COPY(_o, _i) {		\
    _o.x = _i.x;				\
    _o.y = _i.y;				\
    _o.z = _i.z;				\
  }

#define PPRZ_INT32_VECT3_ASSIGN(v, _x, _y, _z) {			\
    v.x = _x;								\
    v.y = _y;								\
    v.z = _z;								\
  }

#define PPRZ_INT32_VECT2_DIFF(_o_i32v2, _i1_i32v2, _i2_i32v2) {	\
    _o_i32v2.x = _i1_i32v2.x - _i2_i32v2.x;			\
    _o_i32v2.y = _i1_i32v2.y - _i2_i32v2.y;			\
  }

#define PPRZ_INT32_EULER_ASSIGN(e, _phi, _theta, _psi) {		\
    e.phi   = _phi;							\
    e.theta = _theta;							\
    e.psi   = _psi;							\
  }

// FIXME
#define PPRZ_INT32_VECT2_OF_LL(_o_i32v2, _i_lla) {	\
    _o_i32v2.x = _i_lla.lat;			\
    _o_i32v2.y = _i_lla.lon;			\
  }

#define PPRZ_INT32_LL_OF_VECT2(_o_lla, _i_i32v2) {	\
    _o_lla.lat = _i_i32v2.x;			\
    _o_lla.lon = _i_i32v2.y;			\
  }


#define BOOZ_INT32_ZERO(_v) {			\
    _v.lat = 0;					\
    _v.lon = 0;					\
    _v.alt = 0;					\
  }

#define PPRZ_INT32_LLA_ASSIGN(_o_lla, _lat, _lon, _alt) {	\
    _o_lla.lat = _lat;			\
    _o_lla.lon = _lon;			\
    _o_lla.alt = _alt;			\
  }

#define PPRZ_INT32_LLA_VECT2_SUM(_o_lla, _i_lla, _i_i32v2) {	\
    _o_lla.lat = _i_lla.lat + _i_i32v2.y;			\
    _o_lla.lon = _i_lla.lon + _i_i32v2.x;			\
  }

#define PPRZ_INT32_LLA_VECT2_DIFF(_o_lla, _i_lla, _i_i32v2) {	\
    _o_lla.lat = _i_lla.lat - _i_i32v2.y;			\
    _o_lla.lon = _i_lla.lon - _i_i32v2.x;			\
  }

#define PPRZ_INT32_LLA_COPY(_o_lla, _i_lla) {	\
    _o_lla.lat = _i_lla.lat;			\
    _o_lla.lon = _i_lla.lon;			\
    _o_lla.alt = _i_lla.alt;			\
  }

#define PPRZ_INT32_LLA_STRIM_LL(_o_lla, _min, _max) {			\
    _o_lla.lat = _o_lla.lat < _min ? _min : _o_lla.lat > _max ? _max : _o_lla.lat; \
    _o_lla.lon = _o_lla.lon < _min ? _min : _o_lla.lon > _max ? _max : _o_lla.lon; \
}

#define PPRZ_INT32_LLA_NORM_LL(_n, _i_lla) {				\
    int32_t n2 = _i_lla.lat*_i_lla.lat + _i_lla.lon*_i_lla.lon;		\
    BOOZ_ISQRT(_n, n2);							\
  }

#define PPRZ_INT32_LLA_SMULT_LL(_o_lla, _i_lla, _s) {	\
    _o_lla.lat = _i_lla.lat * _s;			\
    _o_lla.lon = _i_lla.lon * _s;			\
  }

#define PPRZ_INT32_LLA_SDIV_LL(_o_lla, _i_lla, _s) {	\
    _o_lla.lat = _o_lla.lat / _s;			\
    _o_lla.lon = _o_lla.lon / _s;			\
  }

#define PPRZ_INT32_LLA_SUM_LL(_o_lla, _i1_lla, _i2_lla) { \
    _o_lla.lat =_i1_lla.lat + _i2_lla.lat;			\
    _o_lla.lon =_i1_lla.lon + _i2_lla.lon;			\
  }

#define PPRZ_INT32_LLA_DIFF_LL(_o_lla, _i1_lla, _i2_lla) {			\
    _o_lla.lat =_i1_lla.lat - _i2_lla.lat;			\
    _o_lla.lon =_i1_lla.lon - _i2_lla.lon;			\
  }



#define BOOZ_IQUAT_COPY(_qo, _qi) {					\
    _qo.qi = _qi.qi;							\
    _qo.qx = _qi.qx;							\
    _qo.qy = _qi.qy;							\
    _qo.qz = _qi.qz;							\
  }


#define BOOZ_IQUAT_EXPLEMENTARY(b,a) {		                        \
    b.qi = -a.qi;							\
    b.qx = -a.qx;							\
    b.qy = -a.qy;							\
    b.qz = -a.qz;							\
  }

#define BOOZ_IQUAT_WRAP_SHORTEST(q) {		                        \
    if (q.qi < 0)				                        \
      BOOZ_IQUAT_EXPLEMENTARY(q,q);					\
  }


#define BOOZ_IQUAT_QDOT(qdot, q, r) {		                        \
    qdot.qi = (-r.x*q.qx -r.y*q.qy -r.z*q.qz)>>(IRATE_RES+1);		\
    qdot.qx = ( r.x*q.qi +r.z*q.qy -r.y*q.qz)>>(IRATE_RES+1);		\
    qdot.qy = ( r.y*q.qi -r.z*q.qx +r.x*q.qz)>>(IRATE_RES+1);		\
    qdot.qz = ( r.z*q.qi +r.y*q.qx -r.x*q.qy)>>(IRATE_RES+1);		\
  }


#define BOOZ_IQUAT_QDOT_L(qdot, q, r) {		                        \
    qdot.qi = (-r.x*q.qx -r.y*q.qy -r.z*q.qz)>>(IRATE_RES+1);		\
    qdot.qx = ( r.x*q.qi +r.z*q.qy -r.y*q.qz)>>(IRATE_RES+1);		\
    qdot.qy = ( r.y*q.qi -r.z*q.qx +r.x*q.qz)>>(IRATE_RES+1);		\
    qdot.qz = ( r.z*q.qi +r.y*q.qx -r.x*q.qy)>>(IRATE_RES+1);		\
    int32_t n;								\
    BOOZ_IQUAT_NORM(n, q);						\
    int32_t dn = (1<<IQUAT_RES) - n;					\
    qdot.qi = qdot.qi + ((dn * q.qi)>>IQUAT_RES);			\
    qdot.qx = qdot.qx + ((dn * q.qx)>>IQUAT_RES);			\
    qdot.qy = qdot.qy + ((dn * q.qy)>>IQUAT_RES);			\
    qdot.qz = qdot.qz + ((dn * q.qz)>>IQUAT_RES);			\
  }


/* wrong !!! */
#define BOOZ_IQUAT_INTEG_T(q,qdot,f) {		                        \
    q.qi = q.qi + (qdot.qi+1) / f;					\
    q.qx = q.qx + (qdot.qx+1) / f;					\
    q.qy = q.qy + (qdot.qy+1) / f;					\
    q.qz = q.qz + (qdot.qz+1) / f;					\
  }


#define BOOZ_IQUAT_INTEG(q,qdot,f) {		                        \
    q.qi = q.qi + (qdot.qi) / f;					\
    q.qx = q.qx + (qdot.qx) / f;					\
    q.qy = q.qy + (qdot.qy) / f;					\
    q.qz = q.qz + (qdot.qz) / f;					\
  }





/*
  rotate a vector by the invert of a quaternion 
*/
#define BOOZ_IQUAT_VDIV(v_out, q, v_in) {				\
    const int32_t qi2  = (q.qi*q.qi)>>IQUAT_RES;			\
    const int32_t qx2  = (q.qx*q.qx)>>IQUAT_RES;			\
    const int32_t qy2  = (q.qy*q.qy)>>IQUAT_RES;			\
    const int32_t qz2  = (q.qz*q.qz)>>IQUAT_RES;			\
    const int32_t qiqx = (q.qi*q.qx)>>IQUAT_RES;			\
    const int32_t qiqy = (q.qi*q.qy)>>IQUAT_RES;			\
    const int32_t qiqz = (q.qi*q.qz)>>IQUAT_RES;			\
    const int32_t qxqy = (q.qx*q.qy)>>IQUAT_RES;			\
    const int32_t qxqz = (q.qx*q.qz)>>IQUAT_RES;			\
    const int32_t qyqz = (q.qy*q.qz)>>IQUAT_RES;			\
    const int32_t m00 = qi2 + qx2 - qy2 - qz2;				\
    const int32_t m01 = 2 * (qxqy + qiqz );				\
    const int32_t m02 = 2 * (qxqz - qiqy );				\
    const int32_t m10 = 2 * (qxqy - qiqz );				\
    const int32_t m11 = qi2 - qx2 + qy2 - qz2;				\
    const int32_t m12 = 2 * (qyqz + qiqx );				\
    const int32_t m20 = 2 * (qxqz + qiqy );				\
    const int32_t m21 = 2 * (qyqz - qiqx );				\
    const int32_t m22 = qi2 - qx2 - qy2 + qz2;				\
    v_out.x = (m00 * v_in.x + m10 * v_in.y + m20 * v_in.z)>>IQUAT_RES;	\
    v_out.y = (m01 * v_in.x + m11 * v_in.y + m21 * v_in.z)>>IQUAT_RES;	\
    v_out.z = (m02 * v_in.x + m12 * v_in.y + m22 * v_in.z)>>IQUAT_RES;	\
  }





/* 
   rotate a vector having only a z coordinate by a quaternion 
   same as BOOZ_IQUAT_VMULT with zeros explicitely removed
*/
#define BOOZ_IQUAT_ZVMULT(v_out, q, zv_in) {				\
    const int32_t qi2  = (q.qi*q.qi)>>IQUAT_RES;			\
    const int32_t qx2  = (q.qx*q.qx)>>IQUAT_RES;			\
    const int32_t qy2  = (q.qy*q.qy)>>IQUAT_RES;			\
    const int32_t qz2  = (q.qz*q.qz)>>IQUAT_RES;			\
    const int32_t qiqx = (q.qi*q.qx)>>IQUAT_RES;			\
    const int32_t qiqy = (q.qi*q.qy)>>IQUAT_RES;			\
    const int32_t qxqz = (q.qx*q.qz)>>IQUAT_RES;			\
    const int32_t qyqz = (q.qy*q.qz)>>IQUAT_RES;			\
    const int32_t m02 = 2 * (qxqz - qiqy );				\
    const int32_t m12 = 2 * (qyqz + qiqx );				\
    const int32_t m22 = qi2 - qx2 - qy2 + qz2;				\
    v_out.x = (m02 * zv_in)>>IQUAT_RES;					\
    v_out.y = (m12 * zv_in)>>IQUAT_RES;					\
    v_out.z = (m22 * zv_in)>>IQUAT_RES;					\
  }



#define BOOZ_IVECT2_ASSIGN(v, _x, _y) {					\
    v.x = _x;								\
    v.y = _y;								\
  }

#define BOOZ_IVECT2_ZERO(v) {						\
    BOOZ_IVECT2_ASSIGN(v, 0, 0);					\
  }

#define BOOZ_IVECT2_SUM(c, a, b) {		                        \
    c.x = a.x + b.x;							\
    c.y = a.y + b.y;							\
  }

#define BOOZ_IVECT2_ADD(a, b) {					        \
    a.x += b.x;								\
    a.y += b.y;								\
  }

#define BOOZ_IVECT2_DIFF(_c, _a, _b) {					\
    _c.x = _a.x - _b.x;							\
    _c.y = _a.y - _b.y;							\
  }

#define BOOZ_IVECT2_COPY(_out, _in) {		                        \
    _out.x = _in.x;							\
    _out.y = _in.y;							\
  }

#define BOOZ_IVECT2_NORM(n, v) {					\
    int32_t n2 = v.x*v.x + v.y*v.y;					\
    BOOZ_ISQRT(n, n2);							\
  }

#define BOOZ_IVECT2_SMULT(o, i, s) {					\
    o.x = i.x * s;							\
    o.y = i.y * s;							\
}

#define BOOZ_IVECT2_SDIV(o, i, s) {					\
    o.x = i.x / s;							\
    o.y = i.y / s;							\
}

#define BOOZ_IVECT2_BOUND(_out, _in, _min, _max) {			\
    _out.x = _in.x < _min.x ? _min.x : _in.x > _max.x ? _max.x : _in.x;	\
    _out.y = _in.y < _min.y ? _min.y : _in.y > _max.y ? _max.y : _in.y;	\
  }

#define BOOZ_IVECT2_TRIM(_v, _min, _max) {				\
    _v.x = _v.x < _min.x ? _min.x : _v.x > _max.x ? _max.x : _v.x;	\
    _v.y = _v.y < _min.y ? _min.y : _v.y > _max.y ? _max.y : _v.y;	\
  }

#define BOOZ_IVECT2_STRIM(_v, _min, _max) {				\
    _v.x = _v.x < _min ? _min : _v.x > _max ? _max : _v.x;		\
    _v.y = _v.y < _min ? _min : _v.y > _max ? _max : _v.y;		\
  }


#define BOOZ_IVECT_ASSIGN(v, _x, _y, _z) {				\
    v.x = _x;								\
    v.y = _y;								\
    v.z = _z;								\
  }


#define BOOZ_IVECT_ZERO(v) {						\
    BOOZ_IVECT_ASSIGN(v, 0, 0, 0);					\
  }



#define BOOZ_IVECT_DIFF(c, a, b) {		                        \
    c.x = a.x - b.x;							\
    c.y = a.y - b.y;							\
    c.z = a.z - b.z;							\
  }


#define BOOZ_IVECT_SUM(c, a, b) {		                        \
    c.x = a.x + b.x;							\
    c.y = a.y + b.y;							\
    c.z = a.z + b.z;							\
  }


/* scalar vector multiplication       */
#define BOOZ_IVECT_SMULT(o, i, s) {					\
    o.x = i.x * s;							\
    o.y = i.y * s;							\
    o.z = i.z * s;							\
}

/* Element wise vector multiplication */
#define BOOZ_IVECT_EWMULT(c, a, b, r) {		                        \
    c.x = (a.x * b.x) >> (r);						\
    c.y = (a.y * b.y) >> (r);						\
    c.z = (a.z * b.z) >> (r);						\
  }

#define BOOZ_IVECT_SDIV(o, i, s) {					\
    o.x = i.x / s;							\
    o.y = i.y / s;							\
    o.z = i.z / s;							\
}

#define BOOZ_IVECT_SDIV_ACC(o, i, s) {					\
    o.x = i.x + (i.x>=0 ? s/2 : -s/2);					\
    o.x /=  s;								\
    o.y = i.y + (i.y>=0 ? s/2 : -s/2);					\
    o.y /=  s;								\
    o.z = i.z + (i.z>=0 ? s/2 : -s/2);					\
    o.z /=  s;								\
}


#define BOOZ_IVECT_SUB_EWMULT(o, i, n, s) {				\
    o.x = (i.x - n.x) * s.x;						\
    o.y = (i.y - n.y) * s.y;						\
    o.z = (i.z - n.z) * s.z;						\
}


#define BOOZ_IVECT_CROSS_PRODUCT(vo, v1, v2, r) {			\
    vo.x = (v1.y*v2.z - v1.z*v2.y)>>(r);				\
    vo.y = (v1.z*v2.x - v1.x*v2.z)>>(r);				\
    vo.z = (v1.x*v2.y - v1.y*v2.x)>>(r);				\
  }


#define BOOZ_IVECT_COPY(_out, _in) {		                        \
    _out.x = _in.x;							\
    _out.y = _in.y;							\
    _out.z = _in.z;							\
  }

#include <stdlib.h>
#define BOOZ_IVECT_ABS(_out, _in) {		                        \
    _out.x = abs(_in.x);						\
    _out.y = abs(_in.y);						\
    _out.z = abs(_in.z);						\
  }

#define BOOZ_IVECT_BOUND(_out, _in, _min, _max) {			\
    _out.x = _in.x < _min.x ? _min.x : _in.x > _max.x ? _max.x : _in.x;	\
    _out.y = _in.y < _min.y ? _min.y : _in.y > _max.y ? _max.y : _in.y;	\
    _out.z = _in.z < _min.z ? _min.z : _in.z > _max.z ? _max.z : _in.z;	\
  }


#define BOOZ_IVECT_INTEG(v,vdot,f) {		                        \
    v.x = v.x + (vdot.x) / f;						\
    v.y = v.y + (vdot.y) / f;						\
    v.z = v.z + (vdot.z) / f;						\
  }


#define BOOZ_ISQRT_MAX_ITER 40
#define BOOZ_ISQRT(_out,_in) {			                        \
    if (_in == 0)							\
      _out = 0;								\
    else {								\
      uint32_t s1, s2;							\
      uint8_t iter = 0;							\
      s2 = _in;								\
      do {								\
	s1 = s2;							\
	s2 = _in / s1;							\
	s2 += s1;							\
	s2 /= 2;							\
	iter++;								\
      }									\
      while( ( (s1-s2) > 1) && (iter < BOOZ_ISQRT_MAX_ITER));		\
      _out = s2;							\
    }									\
  }

#include "booz_trig_int.h"

#define BOOZ_ISIN(_s, _a) {						\
    int32_t an = _a;							\
    BOOZ_ANGLE_NORMALIZE(an);						\
    if (an > PI_2_INT) an = PI_INT - an;				\
    else if (an < -PI_2_INT) an = -PI_INT - an;				\
    if (an >= 0) _s = booz_trig_int[an];				\
    else _s = -booz_trig_int[-an];					\
  }


#define BOOZ_ICOS(_c, _a) {						\
    BOOZ_ISIN( _c, _a + PI_2_INT);					\
  }

#define BOOZ_IATAN2(_r, _y, _x) {		                        \
									\
  }


#define BOOZ_IEULER_ZERO(_ie) { \
    _ie.phi   = 0;		\
    _ie.theta = 0;		\
    _ie.psi   = 0;		\
  }


#define BOOZ_IEULER_COPY(_out, _in) {		                        \
    _out.phi   = _in.phi;						\
    _out.theta = _in.theta;						\
    _out.psi   = _in.psi;						\
  }


#define BOOZ_IEULER_INTEG(e,edot,f) {		                        \
    e.phi   = e.phi   + (edot.phi)   / f;				\
    e.theta = e.theta + (edot.theta) / f;				\
    e.psi   = e.psi   + (edot.psi)   / f;				\
  }


#define BOOZ_IEULER_SUM(ec, ea, eb) {		                        \
    ec.phi   = ea.phi   + eb.phi;					\
    ec.theta = ea.theta + eb.theta;					\
    ec.psi   = ea.psi   + eb.psi;					\
  }


#define BOOZ_IEULER_DIFF(ec, ea, eb) {		                        \
    ec.phi   = ea.phi   - eb.phi;					\
    ec.theta = ea.theta - eb.theta;					\
    ec.psi   = ea.psi   - eb.psi;					\
  }


#define BOOZ_IEULER_SDIV(eb, ea, s) {				\
    eb.phi   = ea.phi   / s;					\
    eb.theta = ea.theta / s;					\
    eb.psi   = ea.psi   / s;					\
  }


#define BOOZ_IEULER_BOUND(_out, _in, _min, _max) {			\
    _out.phi   = _in.phi   < _min.phi   ? _min.phi   : _in.phi   > _max.phi   ? _max.phi   : _in.phi;	\
    _out.theta = _in.theta < _min.theta ? _min.theta : _in.theta > _max.theta ? _max.theta : _in.theta;	\
    _out.psi   = _in.psi   < _min.psi   ? _min.psi   : _in.psi   > _max.psi   ? _max.psi   : _in.psi;	\
  }




#endif /* BOOZ_GEOMETRY_INT_H */

