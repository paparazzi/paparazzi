#ifndef PPRZ_ALGEBRA_FLOAT_H
#define PPRZ_ALGEBRA_FLOAT_H


struct FloatVect3 {
  FLOAT_T x;
  FLOAT_T y;
  FLOAT_T z;
};


#define PPRZ_FLOAT_VECT3_COPY(_o, _i) {		\
    _o.x = _i.x;				\
    _o.y = _i.y;				\
    _o.z = _i.z;				\
  }




#endif /* PPRZ_ALGEBRA_FLOAT_H */
