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


#define FLOAT_VECT3_ZERO(_v) {			\
    (_v).x = 0.;				\
    (_v).y = 0.;				\
    (_v).z = 0.;				\
  }

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





/*
 * Quaternions
 */

#define FLOAT_QUAT_ZERO(_q) {						\
    (_q).qi = 1.;							\
    (_q).qx = 0.;							\
    (_q).qy = 0.;							\
    (_q).qz = 0.;							\
  }

#define FLOAT_QUAT_MULT(_c, _a, _b) {					             \
    (_c).qi = (_a).qi*(_b).qi - (_a).qx*(_b).qx - (_a).qy*(_b).qy - (_a).qz*(_b).qz; \
    (_c).qx = (_a).qi*(_b).qx + (_a).qx*(_b).qi + (_a).qy*(_b).qz - (_a).qz*(_b).qy; \
    (_c).qy = (_a).qi*(_b).qy - (_a).qx*(_b).qz + (_a).qy*(_b).qi + (_a).qz*(_b).qx; \
    (_c).qz = (_a).qi*(_b).qz + (_a).qx*(_b).qy - (_a).qy*(_b).qx + (_a).qz*b.qi;    \
  }

#define FLOAT_QUAT_INVERT(_qo, _qi) {					\
    (_qo).qi =  (_qi).qi;						\
    (_qo).qx = -(_qi).qx;						\
    (_qo).qy = -(_qi).qy;						\
    (_qo).qz = -(_qi).qz;						\
  }

#endif /* PPRZ_ALGEBRA_FLOAT_H */
