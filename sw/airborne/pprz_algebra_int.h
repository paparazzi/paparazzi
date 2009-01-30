#ifndef PPRZ_ALGEBRA_INT_H
#define PPRZ_ALGEBRA_INT_H




struct Int32Vect3 {
  int32_t x;
  FLOAT_T y;
  FLOAT_T z;
};


#define PPRZ_INT32_VECT3_COPY(_o, _i) {		\
    _o.x = _i.x;				\
    _o.y = _i.y;				\
    _o.z = _i.z;				\
  }




#endif /* PPRZ_ALGEBRA_INT_H */
