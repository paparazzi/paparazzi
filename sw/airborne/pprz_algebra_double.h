#ifndef PPRZ_ALGEBRA_DOUBLE_H
#define PPRZ_ALGEBRA_DOUBLE_H

#include "pprz_algebra.h"


struct DoubleVect2 {
  FLOAT_T x;
  FLOAT_T y;
};

struct DoubleVect3 {
  FLOAT_T x;
  FLOAT_T y;
  FLOAT_T z;
};

struct DoubleQuat {
  FLOAT_T qi;
  FLOAT_T qx;
  FLOAT_T qy;
  FLOAT_T qz;
};

struct DoubleMat33 {
  FLOAT_T m[3*3];
};

struct DoubleEulers {
  FLOAT_T phi;
  FLOAT_T theta;
  FLOAT_T psi;
};

struct DoubleRates {
  FLOAT_T p;
  FLOAT_T q;
  FLOAT_T r;
};



/* multiply _vin by _mat, store in _vout */
#define DOUBLE_MAT33_VECT3_MUL(_vout, _mat, _vin) {		\
    (_vout).x = (_mat)[0]*(_vin).x + (_mat)[1]*(_vin).y + (_mat)[2]*(_vin).z;	\
    (_vout).y = (_mat)[3]*(_vin).x + (_mat)[4]*(_vin).y + (_mat)[5]*(_vin).z;	\
    (_vout).z = (_mat)[6]*(_vin).x + (_mat)[7]*(_vin).y + (_mat)[8]*(_vin).z;	\
  }

/* multiply _vin by the transpose of _mat, store in _vout */
#define DOUBLE_MAT33_VECT3_TRANSP_MUL(_vout, _mat, _vin) {		\
    (_vout).x = (_mat)[0]*(_vin).x + (_mat)[3]*(_vin).y + (_mat)[6]*(_vin).z;	\
    (_vout).y = (_mat)[1]*(_vin).x + (_mat)[4]*(_vin).y + (_mat)[7]*(_vin).z;	\
    (_vout).z = (_mat)[2]*(_vin).x + (_mat)[5]*(_vin).y + (_mat)[8]*(_vin).z;	\
  }


#endif /* PPRZ_ALGEBRA_DOUBLE_H */
