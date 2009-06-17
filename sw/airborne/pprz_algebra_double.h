#ifndef PPRZ_ALGEBRA_DOUBLE_H
#define PPRZ_ALGEBRA_DOUBLE_H

#include "pprz_algebra.h"


struct DoubleVect2 {
  double x;
  double y;
};

struct DoubleVect3 {
  double x;
  double y;
  double z;
};

struct DoubleQuat {
  double qi;
  double qx;
  double qy;
  double qz;
};

struct DoubleMat33 {
  double m[3*3];
};

struct DoubleEulers {
  double phi;
  double theta;
  double psi;
};

struct DoubleRates {
  double p;
  double q;
  double r;
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

#define DOUBLE_QUAT_OF_EULERS(_q, _e) {					\
    									\
    const double phi2   = (_e).phi/ 2.0;					\
    const double theta2 = (_e).theta/2.0;				\
    const double psi2   = (_e).psi/2.0;					\
									\
    const double s_phi2   = sin( phi2 );				\
    const double c_phi2   = cos( phi2 );				\
    const double s_theta2 = sin( theta2 );				\
    const double c_theta2 = cos( theta2 );				\
    const double s_psi2   = sin( psi2 );				\
    const double c_psi2   = cos( psi2 );				\
									\
    (_q).qi =  c_phi2 * c_theta2 * c_psi2 + s_phi2 * s_theta2 * s_psi2; \
    (_q).qx = -c_phi2 * s_theta2 * s_psi2 + s_phi2 * c_theta2 * c_psi2; \
    (_q).qy =  c_phi2 * s_theta2 * c_psi2 + s_phi2 * c_theta2 * s_psi2; \
    (_q).qz =  c_phi2 * c_theta2 * s_psi2 - s_phi2 * s_theta2 * c_psi2; \
    									\
  }

#endif /* PPRZ_ALGEBRA_DOUBLE_H */
