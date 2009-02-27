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
static void test_5(void);
static void test_6(void);
static void test_7(void);

float test_eulers_of_quat(struct FloatQuat fq, int display);
float test_eulers_of_rmat(struct FloatRMat frm, int display);
float test_rmat_comp(struct FloatRMat ma2b_f, struct FloatRMat mb2c_f, int display);
float test_quat_comp(struct FloatQuat qa2b_f, struct FloatQuat qb2c_f, int display);
float test_rmat_comp_inv(struct FloatRMat ma2c_f, struct FloatRMat mb2c_f, int display);
float test_quat_comp_inv(struct FloatQuat qa2c_f, struct FloatQuat qb2c_f, int display);


int main(int argc, char** argv) {

  //  test_1();
  //  test_2();
  // test_3();
  printf("\n");
  //test_4_int();
  printf("\n");
  //  test_4_float();
  printf("\n");

  test_5();
  test_6();
  test_7();
  return 0;

}

static void test_1(void) {

  struct FloatEulers euler_f = { RadOfDeg(45.), RadOfDeg(0.), RadOfDeg(0.)};
  DISPLAY_FLOAT_EULERS("euler_f", euler_f);

  struct Int32Eulers euler_i;
  EULERS_BFP_OF_REAL(euler_i, euler_f);
  DISPLAY_INT32_EULERS("euler_i", euler_i);
  
  struct FloatQuat quat_f;
  FLOAT_QUAT_OF_EULERS(quat_f, euler_f);
  DISPLAY_FLOAT_QUAT("quat_f", quat_f);

  struct Int32Quat quat_i;
  INT32_QUAT_OF_EULERS(quat_i, euler_i);
  DISPLAY_INT32_QUAT("quat_i", quat_i);

  struct Int32RMat rmat_i;
  INT32_RMAT_OF_QUAT(rmat_i, quat_i);
  DISPLAY_INT32_RMAT("rmat_i", rmat_i);
  
}


static void test_2(void) {
  
  struct Int32Vect3 v1 = { 5000, 5000, 5000 };
  DISPLAY_INT32_VECT3("v1", v1);

  struct FloatEulers euler_f = { RadOfDeg(45.), RadOfDeg(0.), RadOfDeg(0.)};
  DISPLAY_FLOAT_EULERS("euler_f", euler_f);

  struct Int32Eulers euler_i;
  EULERS_BFP_OF_REAL(euler_i, euler_f);
  DISPLAY_INT32_EULERS("euler_i", euler_i);

  struct Int32Quat quat_i;
  INT32_QUAT_OF_EULERS(quat_i, euler_i);
  DISPLAY_INT32_QUAT("quat_i", quat_i);
  INT32_QUAT_NORMALISE(quat_i);
  DISPLAY_INT32_QUAT("quat_i_n", quat_i);

  struct Int32Vect3 v2;
  INT32_QUAT_VMULT(v2, quat_i, v1);
  DISPLAY_INT32_VECT3("v2", v2);

  struct Int32RMat rmat_i;
  INT32_RMAT_OF_QUAT(rmat_i, quat_i);
  DISPLAY_INT32_RMAT("rmat_i", rmat_i);

  struct Int32Vect3 v3;
  INT32_RMAT_VMULT(v3, rmat_i, v1);
  DISPLAY_INT32_VECT3("v3", v3);

  struct Int32RMat rmat_i2;
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
  EULERS_ASSIGN(b2i_e, ANGLE_BFP_OF_REAL(RadOfDeg(10.66)), ANGLE_BFP_OF_REAL(RadOfDeg(-0.7)), ANGLE_BFP_OF_REAL(RadOfDeg(0.)));
  DISPLAY_INT32_EULERS_AS_FLOAT_DEG("b2i_e", b2i_e);

  /* Compute BODY to IMU quaternion */
  struct Int32Quat b2i_q;
  INT32_QUAT_OF_EULERS(b2i_q, b2i_e);
  DISPLAY_INT32_QUAT_AS_EULERS_DEG("b2i_q", b2i_q);
  //  INT32_QUAT_NORMALISE(b2i_q);
  //  DISPLAY_INT32_QUAT_AS_EULERS_DEG("b2i_q_n", b2i_q);

  /* Compute BODY to IMU rotation matrix */
  struct Int32RMat b2i_r;
  INT32_RMAT_OF_EULERS(b2i_r, b2i_e);
  //  DISPLAY_INT32_RMAT("b2i_r", b2i_r);
  DISPLAY_INT32_RMAT_AS_EULERS_DEG("b2i_r", b2i_r);

  /* Compute LTP to IMU eulers */
  struct Int32Eulers l2i_e;
  EULERS_ASSIGN(l2i_e, ANGLE_BFP_OF_REAL(RadOfDeg(0.)), ANGLE_BFP_OF_REAL(RadOfDeg(20.)), ANGLE_BFP_OF_REAL(RadOfDeg(0.)));
  DISPLAY_INT32_EULERS_AS_FLOAT_DEG("l2i_e", l2i_e);

  /* Compute LTP to IMU quaternion */
  struct Int32Quat l2i_q;
  INT32_QUAT_OF_EULERS(l2i_q, l2i_e);
  DISPLAY_INT32_QUAT_AS_EULERS_DEG("l2i_q", l2i_q);
 
  /* Compute LTP to IMU rotation matrix */
  struct Int32RMat l2i_r;
  INT32_RMAT_OF_EULERS(l2i_r, l2i_e);
  //  DISPLAY_INT32_RMAT("l2i_r", l2i_r);
  DISPLAY_INT32_RMAT_AS_EULERS_DEG("l2i_r", l2i_r);


  /* again but from quaternion */
  struct Int32RMat l2i_r2;
  INT32_RMAT_OF_QUAT(l2i_r2, l2i_q);
  //  DISPLAY_INT32_RMAT("l2i_r2", l2i_r2);
  DISPLAY_INT32_RMAT_AS_EULERS_DEG("l2i_r2", l2i_r2);

  /* Compute LTP to BODY quaternion */
  struct Int32Quat l2b_q;
  INT32_QUAT_COMP_INV(l2b_q, b2i_q, l2i_q);
  DISPLAY_INT32_QUAT_AS_EULERS_DEG("l2b_q", l2b_q);

  /* Compute LTP to BODY rotation matrix */
  struct Int32RMat l2b_r;
  INT32_RMAT_COMP_INV(l2b_r, l2i_r, b2i_r);
  //  DISPLAY_INT32_RMAT("l2b_r", l2b_r);
  DISPLAY_INT32_RMAT_AS_EULERS_DEG("l2b_r2", l2b_r);

 /* again but from quaternion */
  struct Int32RMat l2b_r2;
  INT32_RMAT_OF_QUAT(l2b_r2, l2b_q);
  //  DISPLAY_INT32_RMAT("l2b_r2", l2b_r2);
  DISPLAY_INT32_RMAT_AS_EULERS_DEG("l2b_r2", l2b_r2);


  /* compute LTP to BODY eulers */
  struct Int32Eulers l2b_e;
  INT32_EULERS_OF_RMAT(l2b_e, l2b_r);
  DISPLAY_INT32_EULERS_AS_FLOAT_DEG("l2b_e", l2b_e);

  /* again but from quaternion */
  struct Int32Eulers l2b_e2;
  INT32_EULERS_OF_QUAT(l2b_e2, l2b_q);
  DISPLAY_INT32_EULERS_AS_FLOAT_DEG("l2b_e2", l2b_e2);
  
}




static void test_4_int(void) {

  printf("euler to quat to euler - int\n");
  /* initial euler angles */
  struct Int32Eulers _e;
  EULERS_ASSIGN(_e, ANGLE_BFP_OF_REAL(RadOfDeg(-10.66)), ANGLE_BFP_OF_REAL(RadOfDeg(-0.7)), ANGLE_BFP_OF_REAL(RadOfDeg(0.)));
  DISPLAY_INT32_EULERS_AS_FLOAT_DEG("euler orig ", _e);

  /* transform to quaternion */
  struct Int32Quat _q;
  INT32_QUAT_OF_EULERS(_q, _e);
  DISPLAY_INT32_QUAT_AS_EULERS_DEG("quat1 ", _q);
  //  INT32_QUAT_NORMALISE(_q);
  //  DISPLAY_INT32_QUAT_2("_q_n", _q);

  /* back to eulers */
  struct Int32Eulers _e2;
  INT32_EULERS_OF_QUAT(_e2, _q);
  DISPLAY_INT32_EULERS_AS_FLOAT_DEG("back to euler ", _e2);

  
}



static void test_4_float(void) {

  printf("euler to quat to euler - float\n");
  /* initial euler angles */
  struct FloatEulers e;
  EULERS_ASSIGN(e, RadOfDeg(-10.66), RadOfDeg(-0.7), RadOfDeg(0.));
  DISPLAY_FLOAT_EULERS_DEG("e", e);

  /* transform to quaternion */
  struct FloatQuat q;
  FLOAT_QUAT_OF_EULERS(q, e);
  //  DISPLAY_FLOAT_QUAT("q", q);
  FLOAT_QUAT_NORMALISE(q);
  DISPLAY_FLOAT_QUAT("q_n", q);
  DISPLAY_FLOAT_QUAT_AS_INT("q_n as int", q);
  /* back to eulers */
  struct FloatEulers e2;
  FLOAT_EULERS_OF_QUAT(e2, q);
  DISPLAY_FLOAT_EULERS_DEG("e2", e2);


}

static void test_5(void) {

  struct FloatEulers fe;
  EULERS_ASSIGN(fe, RadOfDeg(-10.66), RadOfDeg(-0.7), RadOfDeg(0.));
  DISPLAY_FLOAT_EULERS_DEG("fe", fe);
  printf("\n");
  struct FloatQuat fq;
  FLOAT_QUAT_OF_EULERS(fq, fe);
  test_eulers_of_quat(fq, 1);
  printf("\n");
  struct FloatRMat frm;
  FLOAT_RMAT_OF_EULERS(frm, fe);
  DISPLAY_FLOAT_RMAT_AS_EULERS_DEG("frm", frm);
  test_eulers_of_rmat(frm, 1);
  printf("\n");

}


float test_eulers_of_quat(struct FloatQuat fq, int display) {
  
  struct FloatEulers fe;
  FLOAT_EULERS_OF_QUAT(fe, fq);
  struct Int32Quat iq;
  QUAT_BFP_OF_REAL(iq, fq);
  struct Int32Eulers ie;
  INT32_EULERS_OF_QUAT(ie, iq);
  struct FloatEulers fe2;
  EULERS_FLOAT_OF_BFP(fe2, ie);
  EULERS_SUB(fe2, ie);
  float norm_err = FLOAT_EULERS_NORM(fe2);

  if (display) { 
    printf("euler of quat\n");
    DISPLAY_FLOAT_QUAT("fq", fq);
    DISPLAY_FLOAT_EULERS_DEG("fe", fe);
    DISPLAY_INT32_EULERS("ie", ie);
    DISPLAY_INT32_EULERS_AS_FLOAT_DEG("ieaf", ie);
  }

  return norm_err;
}


float test_eulers_of_rmat(struct FloatRMat frm, int display) {

  struct FloatEulers fe;
  FLOAT_EULERS_OF_RMAT(fe, frm);
  struct Int32RMat irm;
  RMAT_BFP_OF_REAL(irm, frm);
  struct Int32Eulers ie;
  INT32_EULERS_OF_RMAT(ie, irm);
  struct FloatEulers fe2;
  EULERS_FLOAT_OF_BFP(fe2, ie);
  EULERS_SUB(fe2, ie);
  float norm_err = FLOAT_EULERS_NORM(fe2);

  if (display) { 
    printf("euler of rmat\n");
    //    DISPLAY_FLOAT_RMAT("fr", fr);
    DISPLAY_FLOAT_EULERS_DEG("fe", fe);
    DISPLAY_INT32_EULERS("ie", ie);
    DISPLAY_INT32_EULERS_AS_FLOAT_DEG("ieaf", ie);
  }

  return norm_err;

}




static void test_6(void) {

  printf("\n");
  struct FloatEulers ea2b;
  EULERS_ASSIGN(ea2b, RadOfDeg(45.), RadOfDeg(22.), RadOfDeg(0.));
  DISPLAY_FLOAT_EULERS_DEG("ea2b", ea2b);
  struct FloatEulers eb2c;
  EULERS_ASSIGN(eb2c, RadOfDeg(0.), RadOfDeg(0.), RadOfDeg(90.));
  DISPLAY_FLOAT_EULERS_DEG("eb2c", eb2c);
  struct FloatRMat fa2b; 
  FLOAT_RMAT_OF_EULERS(fa2b, ea2b);
  struct FloatRMat fb2c; 
  FLOAT_RMAT_OF_EULERS(fb2c, eb2c);

  printf("\n");
  test_rmat_comp(fa2b, fb2c, 1);

  struct FloatQuat qa2b; 
  FLOAT_QUAT_OF_EULERS(qa2b, ea2b);
  struct FloatQuat qb2c; 
  FLOAT_QUAT_OF_EULERS(qb2c, eb2c);

  printf("\n");
  test_quat_comp(qa2b, qb2c, 1);
  printf("\n");

}



float test_rmat_comp(struct FloatRMat ma2b_f, struct FloatRMat mb2c_f, int display) {

  struct FloatRMat ma2c_f;
  FLOAT_RMAT_COMP(ma2c_f, ma2b_f, mb2c_f);
  struct Int32RMat ma2b_i;
  RMAT_BFP_OF_REAL(ma2b_i, ma2b_f);
  struct Int32RMat mb2c_i;
  RMAT_BFP_OF_REAL(mb2c_i, mb2c_f);
  struct Int32RMat ma2c_i;
  INT32_RMAT_COMP(ma2c_i, ma2b_i, mb2c_i);

  struct FloatRMat err;
  RMAT_DIFF(err, ma2c_f, ma2c_i);
  float norm_err = FLOAT_RMAT_NORM(err);

  if (display) { 
    printf("rmap comp\n");
    DISPLAY_FLOAT_RMAT_AS_EULERS_DEG("a2cf", ma2c_f);
    DISPLAY_INT32_RMAT_AS_EULERS_DEG("a2ci", ma2c_i);
  }
  
  return norm_err;

}


float test_quat_comp(struct FloatQuat qa2b_f, struct FloatQuat qb2c_f, int display) {

  struct FloatQuat qa2c_f;
  FLOAT_QUAT_COMP(qa2c_f, qa2b_f, qb2c_f);
  struct Int32Quat qa2b_i;
  QUAT_BFP_OF_REAL(qa2b_i, qa2b_f);
  struct Int32Quat qb2c_i;
  QUAT_BFP_OF_REAL(qb2c_i, qb2c_f);
  struct Int32Quat qa2c_i;
  INT32_QUAT_COMP(qa2c_i, qa2b_i, qb2c_i);

  struct FloatQuat err;
  QUAT_DIFF(err, qa2c_f, qa2c_i);
  float norm_err = FLOAT_QUAT_NORM(err);

  if (display) { 
    printf("quat comp\n");
    DISPLAY_FLOAT_QUAT_AS_EULERS_DEG("a2cf", qa2c_f);
    DISPLAY_INT32_QUAT_AS_EULERS_DEG("a2ci", qa2c_i);
  }
  
  return norm_err;

}



static void test_7(void) {
  printf("\n");
  struct FloatEulers ea2c;
  EULERS_ASSIGN(ea2c, RadOfDeg(29.742755), RadOfDeg(-40.966522), RadOfDeg(69.467265));
  DISPLAY_FLOAT_EULERS_DEG("ea2c", ea2c);
  
  struct FloatEulers eb2c;
  EULERS_ASSIGN(eb2c, RadOfDeg(0.), RadOfDeg(0.), RadOfDeg(90.));
  DISPLAY_FLOAT_EULERS_DEG("eb2c", eb2c);
  struct FloatRMat fa2c; 
  FLOAT_RMAT_OF_EULERS(fa2c, ea2c);
  struct FloatRMat fb2c; 
  FLOAT_RMAT_OF_EULERS(fb2c, eb2c);

  printf("\n");
  test_rmat_comp_inv(fa2c, fb2c, 1);

  struct FloatQuat qa2c; 
  FLOAT_QUAT_OF_EULERS(qa2c, ea2c);
  struct FloatQuat qb2c; 
  FLOAT_QUAT_OF_EULERS(qb2c, eb2c);

  printf("\n");
  test_quat_comp_inv(qa2c, qb2c, 1);
  printf("\n");



}

float test_rmat_comp_inv(struct FloatRMat ma2c_f, struct FloatRMat mb2c_f, int display) {

  struct FloatRMat ma2b_f;
  FLOAT_RMAT_COMP_INV(ma2b_f, ma2c_f, mb2c_f);
  struct Int32RMat ma2c_i;
  RMAT_BFP_OF_REAL(ma2c_i, ma2c_f);
  struct Int32RMat mb2c_i;
  RMAT_BFP_OF_REAL(mb2c_i, mb2c_f);
  struct Int32RMat ma2b_i;
  INT32_RMAT_COMP_INV(ma2b_i, ma2c_i, mb2c_i);

  struct FloatRMat err;
  RMAT_DIFF(err, ma2b_f, ma2b_i);
  float norm_err = FLOAT_RMAT_NORM(err);

  if (display) { 
    printf("rmap comp_inv\n");
    DISPLAY_FLOAT_RMAT_AS_EULERS_DEG("a2cf", ma2b_f);
    DISPLAY_INT32_RMAT_AS_EULERS_DEG("a2ci", ma2b_i);
  }
  
  return norm_err;

}

float test_quat_comp_inv(struct FloatQuat qa2c_f, struct FloatQuat qb2c_f, int display) {

  struct FloatQuat qa2b_f;
  FLOAT_QUAT_COMP_INV(qa2b_f, qa2c_f, qb2c_f);
  struct Int32Quat qa2c_i;
  QUAT_BFP_OF_REAL(qa2c_i, qa2c_f);
  struct Int32Quat qb2c_i;
  QUAT_BFP_OF_REAL(qb2c_i, qb2c_f);
  struct Int32Quat qa2b_i;
  INT32_QUAT_COMP_INV(qa2b_i, qa2c_i, qb2c_i);

  struct FloatQuat err;
  QUAT_DIFF(err, qa2b_f, qa2b_i);
  float norm_err = FLOAT_QUAT_NORM(err);

  if (display) { 
    printf("quat comp_inv\n");
    DISPLAY_FLOAT_QUAT_AS_EULERS_DEG("a2bf", qa2b_f);
    DISPLAY_INT32_QUAT_AS_EULERS_DEG("a2bi", qa2b_i);
  }
  
  return norm_err;

}
