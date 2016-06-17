/* File : pprz_algebra_float.i */
%module algebra_float
%{
#include "math/pprz_algebra_float.h"
%}

/* don't wrap everything in header
%include "math/pprz_algebra_float.h"
* instead only wrap the structs and extend them with nicer to use methods
*/

struct FloatVect2 {
  float x;
  float y;
};

struct FloatVect3 {
  float x;
  float y;
  float z;
};

struct FloatQuat {
  float qi;
  float qx;
  float qy;
  float qz;
};

struct FloatRMat {
  float m[3 * 3];
};

struct FloatEulers {
  float phi; ///< in radians
  float theta; ///< in radians
  float psi; ///< in radians
};

struct FloatRates {
  float p; ///< in rad/s
  float q; ///< in rad/s
  float r; ///< in rad/s
};

%extend FloatVect2 {
  char *__str__() {
    static char tmp[1024];
    sprintf(tmp,"Vect2(%g, %g)", $self->x ,$self->y);
    return tmp;
  }
  FloatVect2(float x=0.0, float y=0.0) {
    struct FloatVect2 *v = (struct FloatVect2 *) malloc(sizeof(struct FloatVect2));
    v->x = x;
    v->y = y;
    return v;
  }
  struct FloatVect2 __add__(struct FloatVect2 *other) {
    struct FloatVect2 v;
    v.x = $self->x + other->x;
    v.y = $self->y + other->y;
    return v;
  }
  struct FloatVect2 __sub__(struct FloatVect2 *other) {
    struct FloatVect2 v;
    v.x = $self->x - other->x;
    v.y = $self->y - other->y;
    return v;
  }
  float norm2() {
    return float_vect2_norm2($self);
  }
  float norm() {
    return float_vect2_norm($self);
  }
  void normalize() {
    float_vect2_normalize($self);
  }
};

%extend FloatVect3 {
  char *__str__() {
    static char tmp[1024];
    sprintf(tmp,"Vect3(%g, %g, %g)", $self->x ,$self->y, $self->z);
    return tmp;
  }
  FloatVect3(float x=0.0, float y=0.0, float z=0.0) {
    struct FloatVect3 *v = (struct FloatVect3 *) malloc(sizeof(struct FloatVect3));
    v->x = x;
    v->y = y;
    v->z = z;
    return v;
  }
  struct FloatVect3 __add__(struct FloatVect3 *other) {
    struct FloatVect3 v;
    v.x = $self->x + other->x;
    v.y = $self->y + other->y;
    v.z = $self->z + other->z;
    return v;
  }
  struct FloatVect3 __sub__(struct FloatVect3 *other) {
    struct FloatVect3 v;
    v.x = $self->x - other->x;
    v.y = $self->y - other->y;
    v.z = $self->z - other->z;
    return v;
  }
  struct FloatVect3 __rmul__(struct FloatQuat *q) {
    struct FloatVect3 v;
    float_quat_vmult(&v, q, $self);
    return v;
  }
  float norm2() {
    return float_vect3_norm2($self);
  }
  float norm() {
    return float_vect3_norm($self);
  }
  void normalize() {
    float_vect3_normalize($self);
  }
};

%extend FloatQuat {
  char *__str__() {
    static char tmp[1024];
    sprintf(tmp,"Quat(%g, %g, %g, %g)", $self->qi, $self->qx ,$self->qy, $self->qz);
    return tmp;
  }
  FloatQuat(float qi=1.0, float qx=0.0, float qy=0.0, float qz=0.0) {
    struct FloatQuat *v = (struct FloatQuat *) malloc(sizeof(struct FloatQuat));
    v->qi = qi;
    v->qx = qx;
    v->qy = qy;
    v->qz = qz;
    return v;
  }
  struct FloatQuat __mul__(struct FloatQuat *q) {
    struct FloatQuat qout;
    float_quat_comp(&qout, $self, q);
    return qout;
  }
  struct FloatQuat __mul__(struct FloatRMat *rm) {
    struct FloatQuat q;
    float_quat_of_rmat(&q, rm);
    struct FloatQuat qout;
    float_quat_comp(&qout, $self, &q);
    return qout;
  }
  struct FloatVect3 __mul__(struct FloatVect3 *v) {
    struct FloatVect3 vout;
    float_quat_vmult(&vout, $self, v);
    return vout;
  }
  float norm() {
    return float_quat_norm($self);
  }
  void normalize() {
    float_quat_normalize($self);
  }
  void set_identity() {
    float_quat_identity($self);
  }
  void wrap_shortest() {
    float_quat_wrap_shortest($self);
  }
  struct FloatEulers to_eulers() {
    struct FloatEulers e;
    float_eulers_of_quat(&e, $self);
    return e;
  }
  struct FloatRMat to_rmat() {
    struct FloatRMat rm;
    float_rmat_of_quat(&rm, $self);
    return rm;
  }
};

%extend FloatRMat {
  char *__str__() {
    static char tmp[1024];
    sprintf(tmp,"RMat[% .5f, % .5f, % .5f]\n    [% .5f, % .5f, % .5f]\n    [% .5f, % .5f, % .5f]",
            $self->m[0], $self->m[1], $self->m[2], $self->m[3], $self->m[4],
            $self->m[5], $self->m[6], $self->m[7], $self->m[8]);
    return tmp;
  }
  FloatRMat() {
    struct FloatRMat *rm = (struct FloatRMat *) malloc(sizeof(struct FloatRMat));
    float_rmat_identity(rm);
    return rm;
  }
  struct FloatRMat __mul__(struct FloatRMat *m_b2c) {
    struct FloatRMat m_a2c;
    float_rmat_comp(&m_a2c, $self, m_b2c);
    return m_a2c;
  }
  struct FloatRMat __mul__(struct FloatQuat *q) {
    struct FloatRMat m_b2c;
    float_rmat_of_quat(&m_b2c, q);
    struct FloatRMat m_a2c;
    float_rmat_comp(&m_a2c, $self, &m_b2c);
    return m_a2c;
  }
  struct FloatVect3 __mul__(struct FloatVect3 *va) {
    struct FloatVect3 v;
    float_rmat_vmult(&v, $self, va);
    return v;
  }
  void set_identity() {
    float_rmat_identity($self);
  }
  struct FloatRMat inverse() {
    struct FloatRMat inv;
    float_rmat_inv(&inv, $self);
    return inv;
  }
  struct FloatRMat transposed() {
    struct FloatRMat inv;
    float_rmat_inv(&inv, $self);
    return inv;
  }
  /*
  void invert() {
    float_rmat_inv($self, $self);
  }
  void transpose() {
    float_rmat_inv($self, $self);
  }
  */
  float norm() {
    return float_rmat_norm($self);
  }
  struct FloatEulers to_eulers() {
    struct FloatEulers e;
    float_eulers_of_rmat(&e, $self);
    return e;
  }
  struct FloatQuat to_quat() {
    struct FloatQuat q;
    float_quat_of_rmat(&q, $self);
    return q;
  }
  double reorthogonalize() {
    return float_rmat_reorthogonalize($self);
  }
};

%extend FloatEulers {
  char *__str__() {
    static char tmp[1024];
    sprintf(tmp,"Eulers in deg (phi=%g, theta=%g, psi=%g)", DegOfRad($self->phi),
            DegOfRad($self->theta), DegOfRad($self->psi));
    return tmp;
  }
  FloatEulers(float phi=0.0, float theta=0.0, float psi=0.0) {
    struct FloatEulers *v = (struct FloatEulers *) malloc(sizeof(struct FloatEulers));
    v->phi = phi;
    v->theta = theta;
    v->psi = psi;
    return v;
  }
  struct FloatQuat to_quat() {
    struct FloatQuat q;
    float_quat_of_eulers(&q, $self);
    return q;
  }
  struct FloatRMat to_rmat() {
    struct FloatRMat r;
    float_rmat_of_eulers(&r, $self);
    return r;
  }
};

%extend FloatRates {
  char *__str__() {
    static char tmp[1024];
    sprintf(tmp,"Rates(%g, %g, %g)", $self->p ,$self->q, $self->r);
    return tmp;
  }
  FloatRates(float p=0.0, float q=0.0, float r=0.0) {
    struct FloatRates *fr = (struct FloatRates *) malloc(sizeof(struct FloatRates));
    fr->p = p;
    fr->q = q;
    fr->r = r;
    return fr;
  }
  struct FloatRates __add__(struct FloatRates *other) {
    struct FloatRates fr;
    fr.p = $self->p + other->p;
    fr.q = $self->q + other->q;
    fr.r = $self->r + other->r;
    return fr;
  }
  struct FloatRates __sub__(struct FloatRates *other) {
    struct FloatRates fr;
    fr.p = $self->p - other->p;
    fr.q = $self->q - other->q;
    fr.r = $self->r - other->r;
    return fr;
  }
};
