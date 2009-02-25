#ifndef PPRZ_ALGEBRA_PRINT_H
#define PPRZ_ALGEBRA_PRINT_H

#include <stdio.h>


#define DISPLAY_FLOAT_VECT3(text, _v) {				\
    printf("%s %f %f %f\n",text,  (_v).x, (_v).y, (_v).z);	\
  }


#define DISPLAY_FLOAT_EULERS(text, _e) {				\
    printf("%s %f %f %f\n",text,  (_e).phi, (_e).theta, (_e).psi);	\
  }

#define DISPLAY_FLOAT_EULERS_DEG(text, _e) {				\
    printf("%s %f %f %f\n",text,  DegOfRad((_e).phi),			\
	   DegOfRad((_e).theta), DegOfRad((_e).psi));			\
  }



#define DISPLAY_FLOAT_QUAT(text, quat) {				\
    float quat_norm;							\
    FLOAT_QUAT_NORM(quat_norm, quat);					\
    printf("%s %f %f %f %f (%f)\n",text, quat.qi, quat.qx, quat.qy, quat.qz, quat_norm); \
  }



#define DISPLAY_INT32_VECT3(text, _v) {					\
    int32_t norm;							\
    INT32_VECT3_NORM(norm, _v);						\
    printf("%s %d %d %d (%d)\n",text,  (_v).x, (_v).y, (_v).z, norm);	\
  }

#define DISPLAY_INT32_EULERS(text, _e) {				\
    printf("%s %d %d %d\n",text,  (_e).phi, (_e).theta, (_e).psi);	\
  }

#define DISPLAY_INT32_EULERS_2(text, _e) {				\
    printf("%s %d %d %d (%f %f %f)\n",text,				\
	   (_e).phi, (_e).theta, (_e).psi,				\
	   DegOfRad((float)(_e).phi/(1<<INT32_ANGLE_FRAC)),		\
	   DegOfRad((float)(_e).theta/(1<<INT32_ANGLE_FRAC)),		\
	   DegOfRad((float)(_e).psi/(1<<INT32_ANGLE_FRAC)));		\
  }


#define DISPLAY_INT32_QUAT(text, quat) {				\
    int32_t quat_norm;							\
    INT32_QUAT_NORM(quat_norm, quat);					\
    printf("%s %d %d %d %d (%d)\n",text, quat.qi, quat.qx, quat.qy, quat.qz, quat_norm); \
  }


#define DISPLAY_INT32_QUAT_2(text, quat) {				\
    int32_t quat_norm;							\
    INT32_QUAT_NORM(quat_norm, quat);					\
    printf("%s %d %d %d %d (%d) (%f %f %f %f)\n",text,			\
	   quat.qi, quat.qx, quat.qy, quat.qz, quat_norm,		\
	   (float)quat.qi/(1<<INT32_QUAT_FRAC),				\
	   (float)quat.qx/(1<<INT32_QUAT_FRAC),				\
	   (float)quat.qy/(1<<INT32_QUAT_FRAC),				\
	   (float)quat.qz/(1<<INT32_QUAT_FRAC));			\
  }



#define DISPLAY_INT32_RMAT(text, mat) {					\
    printf("%s\n %05d %05d %05d\n %05d %05d %05d\n %05d %05d %05d\n",text, \
	   mat.m[0], mat.m[1], mat.m[2], mat.m[3], mat.m[4], mat.m[5],	\
	   mat.m[6], mat.m[7], mat.m[8]);				\
  }


#endif /* PPRZ_ALGEBRA_PRINT_H */
