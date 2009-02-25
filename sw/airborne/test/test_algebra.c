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

/* euler to quat to euler */
static void test_4_int(void);
static void test_4_float(void);

int main(int argc, char** argv) {

  //  test_1();
  //  test_2();
  //  test_3();
  printf("\n");
  test_4_int();
  printf("\n");
  test_4_float();
  printf("\n");
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

  /* Compute BODY to IMU eulers */
  struct Int32Eulers b2i_e;
  EULERS_ASSIGN(b2i_e, ANGLE_BFP(RadOfDeg(10.66)), ANGLE_BFP(RadOfDeg(-0.7)), ANGLE_BFP(RadOfDeg(0.)));
  DISPLAY_INT32_EULERS_2("b2i_e", b2i_e);

  /* Compute BODY to IMU quaternion */
  struct Int32Quat b2i_q;
  INT32_QUAT_OF_EULERS(b2i_q, b2i_e);
  DISPLAY_INT32_QUAT("b2i_q", b2i_q);
  INT32_QUAT_NORMALISE(b2i_q);
  DISPLAY_INT32_QUAT("b2i_q_n", b2i_q);

  /* Compute BODY to IMU rotation matrix */
  struct Int32Rmat b2i_r;
  INT32_RMAT_OF_EULERS(b2i_r, b2i_e);
  DISPLAY_INT32_RMAT("b2i_r", b2i_r);
  

  /* Compute LTP to IMU eulers */
  struct Int32Eulers atti_e;
  EULERS_ASSIGN(atti_e, ANGLE_BFP(RadOfDeg(0.)), ANGLE_BFP(RadOfDeg(20.)), ANGLE_BFP(RadOfDeg(0.)));
  DISPLAY_INT32_EULERS_2("atti_e", atti_e);

  /* Compute LTP to IMU quaternion */
  struct Int32Quat atti_q;
  INT32_QUAT_OF_EULERS(atti_q, atti_e);
  DISPLAY_INT32_QUAT("atti_q", atti_q);
 
  /* Compute LTP to IMU rotation matrix */
  struct Int32Rmat atti_r;
  INT32_RMAT_OF_EULERS(atti_r, atti_e);
  DISPLAY_INT32_RMAT("atti_r", atti_r);


  /* again but from quaternion */
  struct Int32Rmat atti_r2;
  INT32_RMAT_OF_QUAT(atti_r2, atti_q);
  DISPLAY_INT32_RMAT("atti_r2", atti_r2);


  /* Compute LTP to BODY quaternion */
  struct Int32Quat attb_q;
  INT32_QUAT_DIV(attb_q, b2i_q, atti_q);
  DISPLAY_INT32_QUAT("attb_q", attb_q);

  /* Compute LTP to BODY rotation matrix */
  struct Int32Rmat attb_r;
  //  INT32_RMAT_COMP_INV(attb_r, b2i_r, atti_r);
  INT32_RMAT_COMP_INV(attb_r, atti_r, b2i_r);
  DISPLAY_INT32_RMAT("attb_r", attb_r);

 /* again but from quaternion */
  struct Int32Rmat attb_r2;
  INT32_RMAT_OF_QUAT(attb_r2, attb_q);
  DISPLAY_INT32_RMAT("attb_r2", attb_r2);


  /* compute LTP to BODY eulers */
  struct Int32Eulers attb_e;
  INT32_EULERS_OF_RMAT(attb_e, attb_r);
  DISPLAY_INT32_EULERS_2("attb_e", attb_e);

  /* again but from quaternion */
  struct Int32Eulers attb_e2;
  INT32_EULERS_OF_QUAT(attb_e2, attb_q);
  DISPLAY_INT32_EULERS_2("attb_e2", attb_e2);
  
}




static void test_4_int(void) {

  /* initial euler angles */
  struct Int32Eulers _e;
  EULERS_ASSIGN(_e, ANGLE_BFP(RadOfDeg(10.66)), ANGLE_BFP(RadOfDeg(-0.7)), ANGLE_BFP(RadOfDeg(0.)));
  DISPLAY_INT32_EULERS_2("_e", _e);

  /* trasnform to quaternion */
  struct Int32Quat _q;
  INT32_QUAT_OF_EULERS(_q, _e);
  DISPLAY_INT32_QUAT("_q", _q);
  INT32_QUAT_NORMALISE(_q);
  DISPLAY_INT32_QUAT("_q_n", _q);

  /* back to eulers */
  struct Int32Eulers _e2;
  INT32_EULERS_OF_QUAT(_e2, _q);
  DISPLAY_INT32_EULERS_2("_e2", _e2);


}



static void test_4_float(void) {

  /* initial euler angles */
  struct FloatEulers e;
  EULERS_ASSIGN(e, RadOfDeg(10.66), RadOfDeg(-0.7), RadOfDeg(0.));
  DISPLAY_FLOAT_EULERS_DEG("e", e);

  /* trasnform to quaternion */
  struct FloatQuat q;
  FLOAT_QUAT_OF_EULERS(q, e);
  DISPLAY_FLOAT_QUAT("q", q);
  FLOAT_QUAT_NORMALISE(q);
  DISPLAY_FLOAT_QUAT("q_n", q);

  /* back to eulers */
  struct FloatEulers e2;
  FLOAT_EULERS_OF_QUAT(e2, q);
  DISPLAY_FLOAT_EULERS_DEG("e2", e2);


}
