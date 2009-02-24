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





/* a =  {x, y, z} */
#define VECT3_ASSIGN(_a, _x, _y, _z) {		\
    (_a).x = (_x);				\
    (_a).y = (_y);				\
    (_a).z = (_z);				\
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

/* _vo =  _vi / _s */
#define VECT3_SDIV(_vo, _vi, _s) {			\
    (_vo).x =  (_vi).x / (_s) ;				\
    (_vo).y =  (_vi).y / (_s);				\
    (_vo).z =  (_vi).z / (_s);				\
  }


/*
 * Euler angles
 */

#define EULERS_ASIGN(_e, _phi, _theta, _psi) {		\
    (_e).phi   = _phi;					\
    (_e).theta = _theta;				\
    (_e).psi   = _psi;					\
  }

/* a += b */
#define EULERS_ADD(_a, _b) {				\
    (_a).phi   += (_b).psi;				\
    (_a).theta += (_b).theta;				\
    (_a).psi   += (_b).psi;				\
  }

/* c = a - b */
#define EULERS_DIFF(_c, _a, _b) {		\
    (_c).phi   = (_a).phi   - (_b).phi;		\
    (_c).theta = (_a).theta - (_b).theta;	\
    (_c).psi   = (_a).psi   - (_b).psi;		\
  }


/* _vo =  _vi / _s */
#define EULERS_SDIV(_eo, _ei, _s) {				\
    (_eo).phi   =  (_ei).phi   / (_s) ;				\
    (_eo).theta =  (_ei).theta / (_s);				\
    (_eo).psi   =  (_ei).psi   / (_s);				\
  }



/*
 * Rates
 */

/* a = b */
#define RATES_COPY(_a, _b) {			\
    (_a).p = (_b).p;				\
    (_a).q = (_b).q;				\
    (_a).r = (_b).r;				\
  }

/* a += b */
#define RATES_ADD(_a, _b) {			\
    (_a).p += (_b).p;				\
    (_a).q += (_b).q;				\
    (_a).r += (_b).r;				\
  }

/* c = a - b */
#define RATES_DIFF(_c, _a, _b) {                \
    (_c).p = (_a).p - (_b).p;			\
    (_c).q = (_a).q - (_b).q;			\
    (_c).r = (_a).r - (_b).r;			\
  }

/* _ro =  _ri / _s */
#define RATES_SDIV(_ro, _ri, _s) {		\
    (_ro).p =  (_ri).p / (_s) ;			\
    (_ro).q =  (_ri).q / (_s);			\
    (_ro).r =  (_ri).r / (_s);			\
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

