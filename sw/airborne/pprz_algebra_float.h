#ifndef PPRZ_ALGEBRA_FLOAT_H
#define PPRZ_ALGEBRA_FLOAT_H


struct FloatVect3 {
  FLOAT_T x;
  FLOAT_T y;
  FLOAT_T z;
};

struct FloatMat33 {
  FLOAT_T m[3*3];
};



#define FLOAT_VECT3_COPY(_o, _i) {		\
    (_o).x = (_i).x;				\
    (_o).y = (_i).y;				\
    (_o).z = (_i).z;				\
  }

#define FLOAT_VECT3_DIFF(_c, _a, _b) {		\
    (_c).x = (_a).x - (_b).x;			\
    (_c).y = (_a).y - (_b).y;			\
    (_c).z = (_a).z - (_b).z;			\
  }

#define FLOAT_MAT33_VECT3_MUL(_vout, _mat, _vin) {		\
    (_vout).x = (_mat)[0]*(_vin).x + (_mat)[1]*(_vin).y + (_mat)[2]*(_vin).z;	\
    (_vout).y = (_mat)[3]*(_vin).x + (_mat)[4]*(_vin).y + (_mat)[5]*(_vin).z;	\
    (_vout).z = (_mat)[6]*(_vin).x + (_mat)[7]*(_vin).y + (_mat)[8]*(_vin).z;	\
  }



#endif /* PPRZ_ALGEBRA_FLOAT_H */
