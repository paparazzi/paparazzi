#ifndef PPRZ_ALGEBRA_H
#define PPRZ_ALGEBRA_H

/* a = b */
#define VECT2_COPY(_a, _b) {			\
    (_a).x = (_b).x;				\
    (_a).y = (_b).y;				\
  }

/* a += b */
#define VECT2_ADD(_a, _b) {			\
    (_a).x += (_b).x;				\
    (_a).y += (_b).y;				\
  }


/* c = a - b */
#define VECT2_DIFF(_c, _a, _b) {                \
    (_c).x = (_a).x - (_b).x;			\
    (_c).y = (_a).y - (_b).y;			\
  }


/* _vo = _s * _vi */
#define VECT2_SMUL(_vo, _s, _vi) {			\
    (_vo).x = (_s) * (_vi).x;				\
    (_vo).y = (_s) * (_vi).y;				\
  }

/* _vo =  _vi / _s */
#define VECT2_SDIV(_vo, _s, _vi) {			\
    (_vo).x =  (_vi).x / (_s) ;				\
    (_vo).y =  (_vi).y / (_s);				\
  }





/* a = b */
#define VECT3_COPY(_a, _b) {			\
    (_a).x = (_b).x;				\
    (_a).y = (_b).y;				\
    (_a).z = (_b).z;				\
  }

/* a += b */
#define VECT3_ADD(_a, _b) {			\
    (_a).x += (_b).x;				\
    (_a).y += (_b).y;				\
    (_a).z += (_b).z;				\
  }

/* a -= b */
#define VECT3_SUB(_a, _b) {			\
    (_a).x -= (_b).x;				\
    (_a).y -= (_b).y;				\
    (_a).z -= (_b).z;				\
  }

/* c = a + b */
#define VECT3_SUM(_c, _a, _b) {                 \
    (_c).x = (_a).x + (_b).x;			\
    (_c).y = (_a).y + (_b).y;			\
    (_c).z = (_a).z + (_b).z;			\
  }

/* c = a - b */
#define VECT3_DIFF(_c, _a, _b) {                \
    (_c).x = (_a).x - (_b).x;			\
    (_c).y = (_a).y - (_b).y;			\
    (_c).z = (_a).z - (_b).z;			\
  }

/* _vo = _s * _vi */
#define VECT3_SMUL(_vo, _s, _vi) {			\
    (_vo).x = (_s) * (_vi).x;				\
    (_vo).y = (_s) * (_vi).y;				\
    (_vo).z = (_s) * (_vi).z;				\
  }

/* multiply _vin by _mat, store in _vout */
#define MAT33_VECT3_MUL(_vout, _mat, _vin) {				\
    (_vout).x = (_mat)[0]*(_vin).x + (_mat)[1]*(_vin).y + (_mat)[2]*(_vin).z; \
    (_vout).y = (_mat)[3]*(_vin).x + (_mat)[4]*(_vin).y + (_mat)[5]*(_vin).z; \
    (_vout).z = (_mat)[6]*(_vin).x + (_mat)[7]*(_vin).y + (_mat)[8]*(_vin).z; \
  }

/* multiply _vin by transpose of _mat, store in _vout */
#define MAT33_VECT3_TRANSP_MUL(_vout, _mat, _vin) {			\
    (_vout).x = (_mat)[0]*(_vin).x + (_mat)[3]*(_vin).y + (_mat)[6]*(_vin).z; \
    (_vout).y = (_mat)[1]*(_vin).x + (_mat)[4]*(_vin).y + (_mat)[7]*(_vin).z; \
    (_vout).z = (_mat)[2]*(_vin).x + (_mat)[5]*(_vin).y + (_mat)[8]*(_vin).z; \
  }



#endif /* PPRZ_ALGEBRA_H */

