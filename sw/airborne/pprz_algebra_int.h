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


#if 0
struct Int32Mat33 {
  int32_t m[3*3];
};
#endif
typedef int32_t Int32Mat33[3*3];

#define PPRZ_INT32_VECT3_COPY(_o, _i) {		\
    _o.x = _i.x;				\
    _o.y = _i.y;				\
    _o.z = _i.z;				\
  }

#define PPRZ_INT32_MAT33_VECT3_MUL(_o, _m, _v) {		\
    _o.x = _m[0][0]*_v.x + _m[0][1]*_v.y + _m[0][2]*_v.z;	\
    _o.y = _m[1][0]*_v.x + _m[1][1]*_v.y + _m[1][2]*_v.z;	\
    _o.z = _m[2][0]*_v.x + _m[2][1]*_v.y + _m[2][2]*_v.z;	\
  }

#endif /* PPRZ_ALGEBRA_INT_H */
