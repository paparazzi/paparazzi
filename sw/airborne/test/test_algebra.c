#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "std.h"

#include "pprz_algebra_float.h"
#include "pprz_algebra_int.h"
#include "pprz_algebra_print.h"

static void test_1(void);
static void test_2(void);
static void test_3(void);

int main(int argc, char** argv) {

  //  test_1();
  //  test_2();
  test_3();
  return 0;

}

static void test_1(void) {

  struct FloatEulers euler_f = { RadOfDeg(45.), RadOfDeg(0.), RadOfDeg(0.)};
  DISPLAY_FLOAT_EULERS("euler_f", euler_f);

  struct Int32Eulers euler_i;
  EULERS_ASSIGN(euler_i, ANGLE_BFP(euler_f.phi), ANGLE_BFP(euler_f.theta), ANGLE_BFP(euler_f.psi));
  DISPLAY_INT32_EULERS("euler_i", euler_i);
  
  struct FloatQuat quat_f;
  FLOAT_QUAT_OF_EULERS(quat_f, euler_f);
  DISPLAY_FLOAT_QUAT("quat_f", quat_f);

  struct Int32Quat quat_i;
  INT32_QUAT_OF_EULERS(quat_i, euler_i);
  DISPLAY_INT32_QUAT("quat_i", quat_i);

  struct Int32Rmat rmat_i;
  INT32_RMAT_OF_QUAT(rmat_i, quat_i);
  DISPLAY_INT32_RMAT("rmat_i", rmat_i);
  
}


static void test_2(void) {
  
  struct Int32Vect3 v1 = { 5000, 5000, 5000 };
  DISPLAY_INT32_VECT3("v1", v1);

  struct FloatEulers euler_f = { RadOfDeg(45.), RadOfDeg(0.), RadOfDeg(0.)};
  DISPLAY_FLOAT_EULERS("euler_f", euler_f);

  struct Int32Eulers euler_i;
  EULERS_ASSIGN(euler_i, ANGLE_BFP(euler_f.phi), ANGLE_BFP(euler_f.theta), ANGLE_BFP(euler_f.psi));
  DISPLAY_INT32_EULERS("euler_i", euler_i);

  struct Int32Quat quat_i;
  INT32_QUAT_OF_EULERS(quat_i, euler_i);
  DISPLAY_INT32_QUAT("quat_i", quat_i);
  INT32_QUAT_NORMALISE(quat_i);
  DISPLAY_INT32_QUAT("quat_i_n", quat_i);

  struct Int32Vect3 v2;
  INT32_QUAT_VMULT(v2, quat_i, v1);
  DISPLAY_INT32_VECT3("v2", v2);

  struct Int32Rmat rmat_i;
  INT32_RMAT_OF_QUAT(rmat_i, quat_i);
  DISPLAY_INT32_RMAT("rmat_i", rmat_i);

  struct Int32Vect3 v3;
  INT32_RMAT_VMULT(v3, rmat_i, v1);
  DISPLAY_INT32_VECT3("v3", v3);

  struct Int32Rmat rmat_i2;
  INT32_RMAT_OF_EULERS(rmat_i2, euler_i);
  DISPLAY_INT32_RMAT("rmat_i2", rmat_i2);

  struct Int32Vect3 v4;
  INT32_RMAT_VMULT(v4, rmat_i2, v1);
  DISPLAY_INT32_VECT3("v4", v4);

  struct FloatQuat quat_f;
  FLOAT_QUAT_OF_EULERS(quat_f, euler_f);
  DISPLAY_FLOAT_QUAT("quat_f", quat_f);

  struct FloatVect3 v5;
  VECT3_COPY(v5, v1);
  DISPLAY_FLOAT_VECT3("v5", v5);
  struct FloatVect3 v6;
  FLOAT_QUAT_VMULT(v6, quat_f, v5);
  DISPLAY_FLOAT_VECT3("v6", v6);

}

static void test_3(void) {


}
