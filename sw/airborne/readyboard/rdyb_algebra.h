#ifndef RDYB_ALGEBRA_H
#define RDYB_ALGEBRA_H

#include <inttypes.h>
#include <math.h>

struct Int32Vect3 {
  int32_t x;
  int32_t y;
  int32_t z;
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

struct FloatRMat {
  float m[3*3];
};

struct FloatEuler {
  float phi;
  float theta;
  float psi;
};


/*
 * Misc
 */

#define Square(_a) ((_a)*(_a))

/*
 * Dimension 3 vectors
 */

#define FloatVect3Zero(_a) {			\
    (_a).x = 0.;				\
    (_a).y = 0.;				\
    (_a).z = 0.;				\
  }

#define FloatVect3Norm(_v) (sqrtf(Square((_v).x) + Square((_v).y) + Square((_v).z))) 


#define Vect3Copy(_a, _b) {			\
    (_a).x = (_b).x;				\
    (_a).y = (_b).y;				\
    (_a).z = (_b).z;				\
  }

#define Vect3Diff(_c, _a, _b) {			\
    (_c).x = (_a).x - (_b).x;			\
    (_c).y = (_a).y - (_b).y;			\
    (_c).z = (_a).z - (_b).z;			\
  }

#define Vect3EwDiv(_c, _a, _b) {		\
    (_c).x = (_a).x / (_b).x;			\
    (_c).y = (_a).y / (_b).y;			\
    (_c).z = (_a).z / (_b).z;			\
  }


/*
 * Quaternions
 */

#define FloatQuatZero(_q) {			\
    (_q).qi = 1.;				\
    (_q).qx = 0.;				\
    (_q).qy = 0.;				\
    (_q).qz = 0.;				\
  }

#define FloatQuatNorm(_q) (sqrtf(Square(_q.qi) + Square(_q.qx)+       \
				 Square(_q.qx) + Square(_q.qy)))      \


/*
 * Rotation Matrices
 */

#define FloatRMatZero(_rm) {			\
    (_rm).m[0] = 1.;				\
    (_rm).m[1] = 0.;				\
    (_rm).m[2] = 0.;				\
    (_rm).m[3] = 0.;				\
    (_rm).m[4] = 1.;				\
    (_rm).m[5] = 0.;				\
    (_rm).m[6] = 0.;				\
    (_rm).m[7] = 0.;				\
    (_rm).m[8] = 1.;				\
  }						\

/*
 * Euler angles
 */

#define FloatEulersZero(_eu) {			\
    (_eu).phi   = 0.;				\
    (_eu).theta = 0.;				\
    (_eu).psi   = 0.;				\
  }


#define FloatEulersOfQuat(_e, _q) {					\
                                                                        \
    const float qx2  = (_q).qx*(_q).qx;                                 \
    const float qy2  = (_q).qy*(_q).qy;                                 \
    const float qz2  = (_q).qz*(_q).qz;                                 \
    const float qiqx = (_q).qi*(_q).qx;                                 \
    const float qiqy = (_q).qi*(_q).qy;                                 \
    const float qiqz = (_q).qi*(_q).qz;                                 \
    const float qxqy = (_q).qx*(_q).qy;                                 \
    const float qxqz = (_q).qx*(_q).qz;                                 \
    const float qyqz = (_q).qy*(_q).qz;                                 \
    const float dcm00 = 1.0 - 2.*(  qy2 +  qz2 );                       \
    const float dcm01 =       2.*( qxqy + qiqz );                       \
    const float dcm02 =       2.*( qxqz - qiqy );                       \
    const float dcm12 =       2.*( qyqz + qiqx );                       \
    const float dcm22 = 1.0 - 2.*(  qx2 +  qy2 );                       \
                                                                        \
    (_e).phi = atan2f( dcm12, dcm22 );                                  \
    (_e).theta = -asinf( dcm02 );                                       \
    (_e).psi = atan2f( dcm01, dcm00 );                                  \
                                                                        \
  }

#endif /* RDYB_ALGEBRA_H */

