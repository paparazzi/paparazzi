#ifndef BOOZ_GEOMETRY_FLOAT_H
#define BOOZ_GEOMETRY_FLOAT_H

#include <math.h>

#ifndef RadOfDeg
#define RadOfDeg(d) ( (d)*M_PI/180. )
#define DegOfRad(r) ( (r)/M_PI*180. )
#endif

#if 0
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
#endif




#if 0
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
#endif

#if 0
#define BOOZ_FEULER_ASSIGN_DEG(e, _phi, _theta, _psi) {			\
    BOOZ_FEULER_ASSIGN(e, RadOfDeg(_phi), RadOfDeg(_theta), RadOfDeg(_psi)); \
  }
#endif

#endif /* BOOZ_GEOMETRY_FLOAT_H */

