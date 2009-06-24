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

struct DoubleRMat {
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

#define DOUBLE_VECT3_ROUND(_v) DOUBLE_VECT3_RINT(_v, _v)


#define DOUBLE_VECT3_RINT(_vout, _vin) {	\
    (_vout).x = rint((_vin).x);			\
    (_vout).y = rint((_vin).y);			\
    (_vout).z = rint((_vin).z);			\
  }



#define DOUBLE_RMAT_OF_EULERS(_rm, _e) DOUBLE_RMAT_OF_EULERS_321(_rm, _e)

#define DOUBLE_RMAT_OF_EULERS_321(_rm, _e) {				\
    									\
    const double sphi   = sin((_e).phi);				\
    const double cphi   = cos((_e).phi);				\
    const double stheta = sin((_e).theta);				\
    const double ctheta = cos((_e).theta);				\
    const double spsi   = sin((_e).psi);				\
    const double cpsi   = cos((_e).psi);				\
    									\
    RMAT_ELMT(_rm, 0, 0) = ctheta*cpsi;					\
    RMAT_ELMT(_rm, 0, 1) = ctheta*spsi;					\
    RMAT_ELMT(_rm, 0, 2) = -stheta;					\
    RMAT_ELMT(_rm, 1, 0) = sphi*stheta*cpsi - cphi*spsi;		\
    RMAT_ELMT(_rm, 1, 1) = sphi*stheta*spsi + cphi*cpsi;		\
    RMAT_ELMT(_rm, 1, 2) = sphi*ctheta;					\
    RMAT_ELMT(_rm, 2, 0) = cphi*stheta*cpsi + sphi*spsi;		\
    RMAT_ELMT(_rm, 2, 1) = cphi*stheta*spsi - sphi*cpsi;		\
    RMAT_ELMT(_rm, 2, 2) = cphi*ctheta;					\
    									\
  }




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

#define DOUBLE_EULERS_OF_QUAT(_e, _q) {					\
									\
    const double qx2  = (_q).qx*(_q).qx;				\
    const double qy2  = (_q).qy*(_q).qy;				\
    const double qz2  = (_q).qz*(_q).qz;				\
    const double qiqx = (_q).qi*(_q).qx;				\
    const double qiqy = (_q).qi*(_q).qy;				\
    const double qiqz = (_q).qi*(_q).qz;				\
    const double qxqy = (_q).qx*(_q).qy;				\
    const double qxqz = (_q).qx*(_q).qz;				\
    const double qyqz = (_q).qy*(_q).qz;				\
    const double dcm00 = 1.0 - 2.*(  qy2 +  qz2 );			\
    const double dcm01 =       2.*( qxqy + qiqz );			\
    const double dcm02 =       2.*( qxqz - qiqy );			\
    const double dcm12 =       2.*( qyqz + qiqx );			\
    const double dcm22 = 1.0 - 2.*(  qx2 +  qy2 );			\
									\
    (_e).phi = atan2( dcm12, dcm22 );					\
    (_e).theta = -asin( dcm02 );					\
    (_e).psi = atan2( dcm01, dcm00 );					\
									\
  }

#endif /* PPRZ_ALGEBRA_DOUBLE_H */
