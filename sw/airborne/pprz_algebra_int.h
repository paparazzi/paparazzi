#ifndef PPRZ_ALGEBRA_INT_H
#define PPRZ_ALGEBRA_INT_H

#include "std.h"


struct Int32Vect2 {
  int32_t x;
  int32_t y;
};

struct Int32Vect3 {
  int32_t x;
  int32_t y;
  int32_t z;
};

struct Int64Vect3 {
  int64_t x;
  int64_t y;
  int64_t z;
};



struct Int32Quat {
  int32_t qi;
  int32_t qx;
  int32_t qy;
  int32_t qz;
};

struct Int32Eulers {
  int32_t phi;
  int32_t theta;
  int32_t psi;
};


#if 0
struct Int32Mat33 {
  int32_t m[3*3];
};
#endif
typedef int32_t Int32Mat33[3*3];

#define INT32_VECT3_ZERO(_o) {			\
    _o.x = 0;					\
    _o.y = 0;					\
    _o.z = 0;					\
  }

#define INT32_VECT3_COPY(_o, _i) {		\
    _o.x = _i.x;				\
    _o.y = _i.y;				\
    _o.z = _i.z;				\
  }

#define INT32_VECT3_DIFF(_c, _a, _b) {		\
    _c.x = _a.x - _b.x;				\
    _c.y = _a.y - _b.y;				\
    _c.z = _a.z - _b.z;				\
  }






#define INT32_MAT33_VECT3_MUL(_o, _m, _v) {		\
    _o.x = _m[0][0]*_v.x + _m[0][1]*_v.y + _m[0][2]*_v.z;	\
    _o.y = _m[1][0]*_v.x + _m[1][1]*_v.y + _m[1][2]*_v.z;	\
    _o.z = _m[2][0]*_v.x + _m[2][1]*_v.y + _m[2][2]*_v.z;	\
  }


#define INT32_QUAT_ZERO(_q) {						\
    _q.qi = (1 << IQUAT_RES);						\
    _q.qx = 0;								\
    _q.qy = 0;								\
    _q.qz = 0;								\
  }

#define INT32_QUAT_MULT(c, a, b) {					\
    c.qi = (a.qi*b.qi - a.qx*b.qx - a.qy*b.qy - a.qz*b.qz)>>IQUAT_RES;	\
    c.qx = (a.qi*b.qx + a.qx*b.qi + a.qy*b.qz - a.qz*b.qy)>>IQUAT_RES;	\
    c.qy = (a.qi*b.qy - a.qx*b.qz + a.qy*b.qi + a.qz*b.qx)>>IQUAT_RES;	\
    c.qz = (a.qi*b.qz + a.qx*b.qy - a.qy*b.qx + a.qz*b.qi)>>IQUAT_RES;	\
  }

#define INT32_QUAT_INVERT(_qo, _qi) {					\
    _qo.qi =  _qi.qi;							\
    _qo.qx = -_qi.qx;							\
    _qo.qy = -_qi.qy;							\
    _qo.qz = -_qi.qz;							\
  }



#endif /* PPRZ_ALGEBRA_INT_H */
