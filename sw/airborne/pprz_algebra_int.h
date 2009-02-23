#ifndef PPRZ_ALGEBRA_INT_H
#define PPRZ_ALGEBRA_INT_H


#include "std.h"
#include "pprz_algebra.h"

struct Int32Vect2 {
  int32_t x;
  int32_t y;
};

struct Int32Vect3 {
  int32_t x;
  int32_t y;
  int32_t z;
};

#define INT_32_QUAT_FRAC 15
struct Int32Quat {
  int32_t qi;
  int32_t qx;
  int32_t qy;
  int32_t qz;
};

struct Int32Mat33 {
  int32_t m[3*3];
};

struct Int32Eulers {
  int32_t phi;
  int32_t theta;
  int32_t psi;
};

struct Int32Rates {
  int32_t p;
  int32_t q;
  int32_t r;
};




struct Int64Vect2 {
  int64_t x;
  int64_t y;
};

struct Int64Vect3 {
  int64_t x;
  int64_t y;
  int64_t z;
};

/*
 * Dimension 2 Vectors
 */

#define INT_VECT2_ZERO(_o) {			\
    _o.x = 0;					\
    _o.y = 0;					\
  }



/*
 * Dimension 3 Vectors
 */

#define INT32_VECT3_ASSIGN(v, _x, _y, _z) {     \
    v.x = _x;					\
    v.y = _y;					\
    v.z = _z;					\
  }

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

#define INT32_VECT3_SCALE_2(_a, _b, _num, _den) {	\
    (_a).x = ((_b).x * (_num)) / (_den);		\
    (_a).y = ((_b).y * (_num)) / (_den);		\
    (_a).z = ((_b).z * (_num)) / (_den);		\
  }


/*
 * 3x3 Matrices
 */

#define INT32_MAT33_VECT3_MULT(_o, _m, _v, _f) {			\
    (_o).x = ((_m)[0]*(_v).x + (_m)[1]*(_v).y + (_m)[2]*(_v).z)>>(_f);	\
    (_o).y = ((_m)[3]*(_v).x + (_m)[4]*(_v).y + (_m)[5]*(_v).z)>>(_f);	\
    (_o).z = ((_m)[6]*(_v).x + (_m)[7]*(_v).y + (_m)[8]*(_v).z)>>(_f);	\
  }


/*
 * Quaternions
 */

#define INT32_QUAT_ZERO(_q) {						\
    _q.qi = (1 << INT_32_QUAT_FRAC);					\
    _q.qx = 0;								\
    _q.qy = 0;								\
    _q.qz = 0;								\
  }

#define INT32_QUAT_MULT(c, a, b) {					      \
    c.qi = (a.qi*b.qi - a.qx*b.qx - a.qy*b.qy - a.qz*b.qz)>>INT_32_QUAT_FRAC; \
    c.qx = (a.qi*b.qx + a.qx*b.qi + a.qy*b.qz - a.qz*b.qy)>>INT_32_QUAT_FRAC; \
    c.qy = (a.qi*b.qy - a.qx*b.qz + a.qy*b.qi + a.qz*b.qx)>>INT_32_QUAT_FRAC; \
    c.qz = (a.qi*b.qz + a.qx*b.qy - a.qy*b.qx + a.qz*b.qi)>>INT_32_QUAT_FRAC; \
  }

#define INT32_QUAT_INVERT(_qo, _qi) {					\
    (_qo).qi =  (_qi).qi;						\
    (_qo).qx = -(_qi).qx;						\
    (_qo).qy = -(_qi).qy;						\
    (_qo).qz = -(_qi).qz;						\
  }



#endif /* PPRZ_ALGEBRA_INT_H */
