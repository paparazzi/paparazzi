#ifndef PPRZ_GEODETIC_H
#define PPRZ_GEODETIC_H


#define ENU_OF_TO_NED(_po, _pi) {		\
    (_po).x =  (_pi).y;				\
    (_po).y =  (_pi).x;				\
    (_po).z = -(_pi).z;				\
  }


#endif /* PPRZ_GEODETIC_H */
