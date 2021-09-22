#ifndef PPRZ_ALGEBRA_PRINT_H
#define PPRZ_ALGEBRA_PRINT_H

#include <stdio.h>


#define DISPLAY_FLOAT_VECT3(text, _v) {       \
    printf("%s %f %f %f\n",text,  (_v).x, (_v).y, (_v).z);  \
  }

#define DISPLAY_FLOAT_RATES(text, _v) {       \
    printf("%s %f %f %f\n",text,  (_v).p, (_v).q, (_v).r);  \
  }

#define DISPLAY_FLOAT_RATES_DEG(text, _v) {       \
    printf("%s %f %f %f\n",text,  DegOfRad((_v).p), DegOfRad((_v).q), DegOfRad((_v).r)); \
  }

#define DISPLAY_FLOAT_RMAT(text, mat) {         \
    printf("%s\n %f %f %f\n %f %f %f\n %f %f %f\n",text, \
           mat.m[0], mat.m[1], mat.m[2], mat.m[3], mat.m[4], mat.m[5],  \
           mat.m[6], mat.m[7], mat.m[8]);       \
  }

#define DISPLAY_FLOAT_EULERS(text, _e) {        \
    printf("%s %f %f %f\n",text,  (_e).phi, (_e).theta, (_e).psi);  \
  }

#define DISPLAY_FLOAT_EULERS_DEG(text, _e) {        \
    printf("%s %f %f %f\n",text,  DegOfRad((_e).phi),     \
           DegOfRad((_e).theta), DegOfRad((_e).psi));     \
  }



#define DISPLAY_FLOAT_QUAT(text, quat) {        \
    float quat_norm = float_quat_norm(&quat);        \
    printf("%s %f %f %f %f (%f)\n",text, quat.qi, quat.qx, quat.qy, quat.qz, quat_norm); \
  }

#define DISPLAY_FLOAT_QUAT_AS_INT(text, quat) {       \
    float quat_norm = float_quat_norm(&quat);        \
    struct Int32Quat qi;            \
    QUAT_BFP_OF_REAL(qi, quat);           \
    printf("%s %d %d %d %d (%f)\n",text, qi.qi, qi.qx, qi.qy, qi.qz, quat_norm); \
  }

#define DISPLAY_FLOAT_QUAT_AS_EULERS_DEG(text, quat) {      \
    struct FloatEulers _fe;           \
    float_eulers_of_quat(&_fe, &quat);          \
    DISPLAY_FLOAT_EULERS_DEG(text, _fe);        \
  }



#define DISPLAY_FLOAT_RMAT_AS_EULERS_DEG(text, _mat) {      \
    struct FloatEulers _fe;           \
    float_eulers_of_rmat(&_fe, &(_mat));          \
    DISPLAY_FLOAT_EULERS_DEG(text, _fe);        \
  }






#define DISPLAY_INT32_VECT3(text, _v) {         \
    int32_t norm = INT32_VECT3_NORM(_v);            \
    printf("%s %d %d %d (%d)\n",text,  (_v).x, (_v).y, (_v).z, norm); \
  }

#define DISPLAY_INT32_RATES(text, _v) {         \
    printf("%s %d %d %d\n",text,  (_v).p, (_v).q, (_v).r);    \
  }

#define DISPLAY_INT32_RATES_AS_FLOAT(text, _r) {      \
    struct FloatRates _fr;            \
    RATES_FLOAT_OF_BFP(_fr, (_r));          \
    DISPLAY_FLOAT_RATES(text, _fr);         \
  }

#define DISPLAY_INT32_RATES_AS_FLOAT_DEG(text, _r) {      \
    struct FloatRates _fr;            \
    RATES_FLOAT_OF_BFP(_fr, (_r));          \
    DISPLAY_FLOAT_RATES_DEG(text, _fr);         \
  }



#define DISPLAY_INT32_EULERS(text, _e) {        \
    printf("%s %d %d %d\n",text,  (_e).phi, (_e).theta, (_e).psi);  \
  }

#define DISPLAY_INT32_EULERS_AS_FLOAT(text, _ie) {      \
    struct FloatEulers _fe;           \
    EULERS_FLOAT_OF_BFP(_fe, (_ie));        \
    DISPLAY_FLOAT_EULERS(text, _fe);          \
  }


#define DISPLAY_INT32_EULERS_AS_FLOAT_DEG(text, _ie) {      \
    struct FloatEulers _fe;           \
    EULERS_FLOAT_OF_BFP(_fe, (_ie));          \
    DISPLAY_FLOAT_EULERS_DEG(text, _fe);        \
  }



#define DISPLAY_INT32_QUAT(text, quat) {        \
    int32_t quat_norm;              \
    INT32_QUAT_NORM(quat_norm, quat);         \
    printf("%s %d %d %d %d (%d)\n",text, quat.qi, quat.qx, quat.qy, quat.qz, quat_norm); \
  }


#define DISPLAY_INT32_QUAT_2(text, quat) {        \
    int32_t quat_norm;              \
    INT32_QUAT_NORM(quat_norm, quat);         \
    printf("%s %d %d %d %d (%d) (%f %f %f %f)\n",text,      \
           quat.qi, quat.qx, quat.qy, quat.qz, quat_norm,   \
           (float)quat.qi/(1<<INT32_QUAT_FRAC),       \
           (float)quat.qx/(1<<INT32_QUAT_FRAC),       \
           (float)quat.qy/(1<<INT32_QUAT_FRAC),       \
           (float)quat.qz/(1<<INT32_QUAT_FRAC));      \
  }

#define DISPLAY_INT32_QUAT_AS_EULERS_DEG(text, _quat) {     \
    struct FloatQuat _fq;           \
    QUAT_FLOAT_OF_BFP(_fq, _quat);          \
    struct FloatEulers _fe;           \
    float_eulers_of_quat(&_fe, &_fq);         \
    DISPLAY_FLOAT_EULERS_DEG(text, _fe);        \
  }


#define DISPLAY_INT32_RMAT(text, mat) {         \
    printf("%s\n %05d %05d %05d\n %05d %05d %05d\n %05d %05d %05d\n",text, \
           mat.m[0], mat.m[1], mat.m[2], mat.m[3], mat.m[4], mat.m[5],  \
           mat.m[6], mat.m[7], mat.m[8]);       \
  }


#define DISPLAY_INT32_RMAT_AS_FLOAT(text, mat) {      \
    printf("%s\n %f %f %f\n %f %f %f\n %f %f %f\n",text, \
           TRIG_FLOAT_OF_BFP(mat.m[0]), TRIG_FLOAT_OF_BFP(mat.m[1]),  \
           TRIG_FLOAT_OF_BFP(mat.m[2]), TRIG_FLOAT_OF_BFP(mat.m[3]),  \
           TRIG_FLOAT_OF_BFP(mat.m[4]), TRIG_FLOAT_OF_BFP(mat.m[5]),  \
           TRIG_FLOAT_OF_BFP(mat.m[6]), TRIG_FLOAT_OF_BFP(mat.m[7]),  \
           TRIG_FLOAT_OF_BFP(mat.m[8]));        \
  }


#define DISPLAY_INT32_RMAT_AS_EULERS_DEG(text, _mat) {      \
    struct FloatRMat _frm;            \
    RMAT_FLOAT_OF_BFP(_frm, (_mat));          \
    struct FloatEulers _fe;           \
    float_eulers_of_rmat(&_fe, &_frm);          \
    DISPLAY_FLOAT_EULERS_DEG(text, _fe);        \
  }

#endif /* PPRZ_ALGEBRA_PRINT_H */
