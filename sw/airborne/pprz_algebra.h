#ifndef PPRZ_ALGEBRA_H
#define PPRZ_ALGEBRA_H

#define SQUARE(_a) ((_a)*(_a))

/*
 * Dimension 2 vectors
 */

/* a =  {x, y} */
#define VECT2_ASSIGN(_a, _x, _y) {		\
    (_a).x = (_x);				\
    (_a).y = (_y);				\
  }

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

/* a -= b */
#define VECT2_SUB(_a, _b) {			\
    (_a).x -= (_b).x;				\
    (_a).y -= (_b).y;				\
  }

/* c = a + b */
#define VECT2_SUM(_c, _a, _b) {			\
    (_c).x = (_a).x + (_b).x;			\
    (_c).y = (_a).y + (_b).y;			\
  }

/* c = a - b */
#define VECT2_DIFF(_c, _a, _b) {                \
    (_c).x = (_a).x - (_b).x;			\
    (_c).y = (_a).y - (_b).y;			\
  }

/* _vo = _s * _vi */
#define VECT2_SMUL(_vo, _s, _vi) {		\
    (_vo).x = (_s) * (_vi).x;			\
    (_vo).y = (_s) * (_vi).y;			\
  }

/* _vo =  _vi / _s */
#define VECT2_SDIV(_vo, _s, _vi) {		\
    (_vo).x =  (_vi).x / (_s);			\
    (_vo).y =  (_vi).y / (_s);			\
  }

/* _v = Bound(_v, _min, _max) */
#define VECT2_STRIM(_v, _min, _max) {				\
    (_v).x = (_v).x < _min ? _min : (_v).x > _max ? _max : (_v).x;		\
    (_v).y = (_v).y < _min ? _min : (_v).y > _max ? _max : (_v).y;		\
  }



/*
 * Dimension 3 vectors
 */

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
    (_vo).x =  (_vi).x / (_s);				\
    (_vo).y =  (_vi).y / (_s);				\
    (_vo).z =  (_vi).z / (_s);				\
  }

/* _v = Bound(_v, _min, _max) */
#define VECT3_STRIM(_v, _min, _max) {				\
    (_v).x = (_v).x < _min ? _min : (_v).x > _max ? _max : (_v).x;		\
    (_v).y = (_v).y < _min ? _min : (_v).y > _max ? _max : (_v).y;		\
    (_v).z = (_v).z < _min ? _min : (_v).z > _max ? _max : (_v).z;		\
  }

/*  */
#define VECT3_EW_DIV(_vo, _va, _vb) {				\
    (_vo).x =  (_va).x / (_vb).x;				\
    (_vo).y =  (_va).y / (_vb).y;				\
    (_vo).z =  (_va).z / (_vb).z;				\
  } 

/*
 * Euler angles
 */

#define EULERS_COPY(_a, _b) {				\
    (_a).phi   = (_b).phi;				\
    (_a).theta = (_b).theta;				\
    (_a).psi   = (_b).psi;				\
  }

#define EULERS_ASSIGN(_e, _phi, _theta, _psi) {		\
    (_e).phi   = _phi;					\
    (_e).theta = _theta;				\
    (_e).psi   = _psi;					\
  }

/* a += b */
#define EULERS_ADD(_a, _b) {				\
    (_a).phi   += (_b).phi;				\
    (_a).theta += (_b).theta;				\
    (_a).psi   += (_b).psi;				\
  }

/* a += b */
#define EULERS_SUB(_a, _b) {				\
    (_a).phi   -= (_b).phi;				\
    (_a).theta -= (_b).theta;				\
    (_a).psi   -= (_b).psi;				\
  }

/* c = a - b */
#define EULERS_DIFF(_c, _a, _b) {		\
    (_c).phi   = (_a).phi   - (_b).phi;		\
    (_c).theta = (_a).theta - (_b).theta;	\
    (_c).psi   = (_a).psi   - (_b).psi;		\
  }


/* _vo =  _vi / _s */
#define EULERS_SDIV(_eo, _ei, _s) {				\
    (_eo).phi   =  (_ei).phi   / (_s);				\
    (_eo).theta =  (_ei).theta / (_s);				\
    (_eo).psi   =  (_ei).psi   / (_s);				\
  }


/*
 * Rates
 */

/* ra =  {p, q, r} */
#define RATES_ASSIGN(_ra, _p, _q, _r) {		\
    (_ra).p = (_p);				\
    (_ra).q = (_q);				\
    (_ra).r = (_r);				\
  }

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

/* a -= b */
#define RATES_SUB(_a, _b) {			\
    (_a).p -= (_b).p;				\
    (_a).q -= (_b).q;				\
    (_a).r -= (_b).r;				\
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

/* Element wise vector multiplication */
#define RATES_EWMULT_RSHIFT(c, a, b, _s) {				\
    c.p = (a.p * b.p) >> (_s);						\
    c.q = (a.q * b.q) >> (_s);						\
    c.r = (a.r * b.r) >> (_s);						\
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

#define QUAT_DIFF(_qc, _qa, _qb) {				\
    (_qc).qi = (_qa).qi - (_qb).qi;				\
    (_qc).qx = (_qa).qx - (_qb).qx;				\
    (_qc).qy = (_qa).qy - (_qb).qy;				\
    (_qc).qz = (_qa).qz - (_qb).qz;				\
  }

#define QUAT_COPY(_qo, _qi) {			\
    (_qo).qi = (_qi).qi;			\
    (_qo).qx = (_qi).qx;			\
    (_qo).qy = (_qi).qy;			\
    (_qo).qz = (_qi).qz;			\
  }

#define RMAT_DIFF(_c, _a, _b) {					\
    (_c).m[0] = (_a).m[0] - (_b).m[0];				\
    (_c).m[1] = (_a).m[1] - (_b).m[1];				\
    (_c).m[2] = (_a).m[2] - (_b).m[2];				\
    (_c).m[3] = (_a).m[3] - (_b).m[3];				\
    (_c).m[4] = (_a).m[4] - (_b).m[4];				\
    (_c).m[5] = (_a).m[5] - (_b).m[5];				\
    (_c).m[6] = (_a).m[6] - (_b).m[6];				\
    (_c).m[7] = (_a).m[7] - (_b).m[7];				\
    (_c).m[8] = (_a).m[8] - (_b).m[8];				\
  }





// if defined PPRZ_ALGEBRA_INT_H && defined PPRZ_ALGEBRA_FLOAT
//#include "pprz_algebra_int.h"
//#include "pprz_algebra_float.h"


#define EULERS_FLOAT_OF_BFP(_ef, _ei) {				\
    (_ef).phi   = ANGLE_FLOAT_OF_BFP((_ei).phi);		\
    (_ef).theta = ANGLE_FLOAT_OF_BFP((_ei).theta);		\
    (_ef).psi   = ANGLE_FLOAT_OF_BFP((_ei).psi);		\
  }

#define EULERS_BFP_OF_REAL(_ei, _ef) {			\
    (_ei).phi   = ANGLE_BFP_OF_REAL((_ef).phi);		\
    (_ei).theta = ANGLE_BFP_OF_REAL((_ef).theta);	\
    (_ei).psi   = ANGLE_BFP_OF_REAL((_ef).psi);		\
  }

#define RMAT_BFP_OF_REAL(_ei, _ef) {			\
    (_ei).m[0]   = TRIG_BFP_OF_REAL((_ef).m[0]);		\
    (_ei).m[1]   = TRIG_BFP_OF_REAL((_ef).m[1]);		\
    (_ei).m[2]   = TRIG_BFP_OF_REAL((_ef).m[2]);		\
    (_ei).m[3]   = TRIG_BFP_OF_REAL((_ef).m[3]);		\
    (_ei).m[4]   = TRIG_BFP_OF_REAL((_ef).m[4]);		\
    (_ei).m[5]   = TRIG_BFP_OF_REAL((_ef).m[5]);		\
    (_ei).m[6]   = TRIG_BFP_OF_REAL((_ef).m[6]);		\
    (_ei).m[7]   = TRIG_BFP_OF_REAL((_ef).m[7]);		\
    (_ei).m[8]   = TRIG_BFP_OF_REAL((_ef).m[8]);		\
  }

#define RMAT_FLOAT_OF_BFP(_ef, _ei) {				\
    (_ef).m[0]   = TRIG_FLOAT_OF_BFP((_ei).m[0]);		\
    (_ef).m[1]   = TRIG_FLOAT_OF_BFP((_ei).m[1]);		\
    (_ef).m[2]   = TRIG_FLOAT_OF_BFP((_ei).m[2]);		\
    (_ef).m[3]   = TRIG_FLOAT_OF_BFP((_ei).m[3]);		\
    (_ef).m[4]   = TRIG_FLOAT_OF_BFP((_ei).m[4]);		\
    (_ef).m[5]   = TRIG_FLOAT_OF_BFP((_ei).m[5]);		\
    (_ef).m[6]   = TRIG_FLOAT_OF_BFP((_ei).m[6]);		\
    (_ef).m[7]   = TRIG_FLOAT_OF_BFP((_ei).m[7]);		\
    (_ef).m[8]   = TRIG_FLOAT_OF_BFP((_ei).m[8]);		\
  }



#define QUAT_FLOAT_OF_BFP(_qf, _qi) {			\
    (_qf).qi = QUAT1_FLOAT_OF_BFP((_qi).qi);		\
    (_qf).qx = QUAT1_FLOAT_OF_BFP((_qi).qx);		\
    (_qf).qy = QUAT1_FLOAT_OF_BFP((_qi).qy);		\
    (_qf).qz = QUAT1_FLOAT_OF_BFP((_qi).qz);		\
  }

#define QUAT_BFP_OF_REAL(_qi, _qf) {			\
    (_qi).qi = QUAT1_BFP_OF_REAL((_qf).qi);		\
    (_qi).qx = QUAT1_BFP_OF_REAL((_qf).qx);		\
    (_qi).qy = QUAT1_BFP_OF_REAL((_qf).qy);		\
    (_qi).qz = QUAT1_BFP_OF_REAL((_qf).qz);		\
  }

#define RATES_FLOAT_OF_BFP(_ef, _ei) {				\
    (_ef).phi   = RATE_FLOAT_OF_BFP((_ei).phi);		\
    (_ef).theta = RATE_FLOAT_OF_BFP((_ei).theta);		\
    (_ef).psi   = RATE_FLOAT_OF_BFP((_ei).psi);		\
  }

#define RATES_BFP_OF_REAL(_ei, _ef) {			\
    (_ei).phi   = RATE_BFP_OF_REAL((_ef).phi);		\
    (_ei).theta = RATE_BFP_OF_REAL((_ef).theta);	\
    (_ei).psi   = RATE_BFP_OF_REAL((_ef).psi);		\
  }


#endif /* PPRZ_ALGEBRA_H */

