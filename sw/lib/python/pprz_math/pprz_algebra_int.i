/* File : pprz_algebra_int.i */
%module algebra_int
%include "stdint.i"
%{
#include "math/pprz_algebra_int.h"
%}

/* don't wrap everything in header
%include "math/pprz_algebra_int.h"
* instead only wrap the structs and extend them with nicer to use methods
*/

struct Int32Vect2 {
  int32_t x;
  int32_t y;
};

struct Int32Vect3 {
  int32_t x;
  int32_t y;
  int32_t z;
};

struct Int32Quat {
  int32_t qi;
  int32_t qx;
  int32_t qy;
  int32_t qz;
};

struct Int32RMat {
  int32_t m[3 * 3];
};

struct Int32Eulers {
  int32_t phi;   ///< in rad with #INT32_ANGLE_FRAC
  int32_t theta; ///< in rad with #INT32_ANGLE_FRAC
  int32_t psi;   ///< in rad with #INT32_ANGLE_FRAC
};

extern uint32_t int32_sqrt(uint32_t in);

%extend Int32Vect2 {
  char *__str__() {
    static char tmp[1024];
    sprintf(tmp,"Vect2(%d, %d)", $self->x ,$self->y);
    return tmp;
  }
  Int32Vect2(int32_t x=0, int32_t y=0) {
    struct Int32Vect2 *v = (struct Int32Vect2 *) malloc(sizeof(struct Int32Vect2));
    v->x = x;
    v->y = y;
    return v;
  }
  struct Int32Vect2 __add__(struct Int32Vect2 *other) {
    struct Int32Vect2 v;
    v.x = $self->x + other->x;
    v.y = $self->y + other->y;
    return v;
  }
  struct Int32Vect2 __sub__(struct Int32Vect2 *other) {
    struct Int32Vect2 v;
    v.x = $self->x - other->x;
    v.y = $self->y - other->y;
    return v;
  }
  int32_t norm2() {
    return int32_vect2_norm2($self);
  }
  int32_t norm() {
    return int32_vect2_norm($self);
  }
  void normalize(uint8_t frac) {
    int32_vect2_normalize($self, frac);
  }
};

%extend Int32Vect3 {
  char *__str__() {
    static char tmp[1024];
    sprintf(tmp,"Vect3(%d, %d, %d)", $self->x ,$self->y, $self->z);
    return tmp;
  }
  Int32Vect3(int32_t x=0, int32_t y=0, int32_t z=0) {
    struct Int32Vect3 *v = (struct Int32Vect3 *) malloc(sizeof(struct Int32Vect3));
    v->x = x;
    v->y = y;
    v->z = z;
    return v;
  }
  struct Int32Vect3 __add__(struct Int32Vect3 *other) {
    struct Int32Vect3 v;
    v.x = $self->x + other->x;
    v.y = $self->y + other->y;
    v.z = $self->z + other->z;
    return v;
  }
  struct Int32Vect3 __sub__(struct Int32Vect3 *other) {
    struct Int32Vect3 v;
    v.x = $self->x - other->x;
    v.y = $self->y - other->y;
    v.z = $self->z - other->z;
    return v;
  }
  struct Int32Vect3 __rmul__(struct Int32Quat *q) {
    struct Int32Vect3 v;
    int32_quat_vmult(&v, q, $self);
    return v;
  }
  int32_t norm2() {
    return VECT3_NORM2(*$self);
  }
  int32_t norm() {
    return int32_sqrt(VECT3_NORM2(*$self));
  }
  void normalize(uint8_t frac) {
    const int32_t n = int32_sqrt(VECT3_NORM2(*$self) >> frac);
    if (n > 0) {
      const int32_t f = BFP_OF_REAL((1.), frac);
      $self->x = $self->x * f / (int32_t)n;
      $self->y = $self->y * f / (int32_t)n;
      $self->z = $self->z * f / (int32_t)n;
    }
  }
  struct Int32Vect3 normalized(uint8_t frac) {
    struct Int32Vect3 v;
    const int32_t n = int32_sqrt(VECT3_NORM2(*$self) >> frac);
    if (n > 0) {
      const int32_t f = BFP_OF_REAL((1.), frac);
      v.x = $self->x * f / (int32_t)n;
      v.y = $self->y * f / (int32_t)n;
      v.z = $self->z * f / (int32_t)n;
    }
    return v;
  }
};

%extend Int32Quat {
  char *__str__() {
    static char tmp[1024];
    sprintf(tmp,"Quat(%d, %d, %d, %d)", $self->qi, $self->qx ,$self->qy, $self->qz);
    return tmp;
  }
  Int32Quat(int32_t qi=1.0, int32_t qx=0, int32_t qy=0, int32_t qz=0) {
    struct Int32Quat *v = (struct Int32Quat *) malloc(sizeof(struct Int32Quat));
    v->qi = qi;
    v->qx = qx;
    v->qy = qy;
    v->qz = qz;
    return v;
  }
  struct Int32Quat __mul__(struct Int32Quat *q) {
    struct Int32Quat qout;
    int32_quat_comp(&qout, $self, q);
    return qout;
  }
  struct Int32Quat __mul__(struct Int32RMat *rm) {
    struct Int32Quat q;
    int32_quat_of_rmat(&q, rm);
    struct Int32Quat qout;
    int32_quat_comp(&qout, $self, &q);
    return qout;
  }
  int32_t norm() {
    return int32_quat_norm($self);
  }
  void normalize() {
    int32_quat_normalize($self);
  }
  void set_identity() {
    int32_quat_identity($self);
  }
  void wrap_shortest() {
    int32_quat_wrap_shortest($self);
  }
  struct Int32Eulers to_eulers() {
    struct Int32Eulers e;
    int32_eulers_of_quat(&e, $self);
    return e;
  }
  struct Int32RMat to_rmat() {
    struct Int32RMat rm;
    int32_rmat_of_quat(&rm, $self);
    return rm;
  }
};

%extend Int32RMat {
  char *__str__() {
    static char tmp[1024];
    sprintf(tmp,"RMat[% d, % d, % d]\n    [% d, % d, % d]\n    [% d, % d, % d]",
            $self->m[0], $self->m[1], $self->m[2], $self->m[3], $self->m[4],
            $self->m[5], $self->m[6], $self->m[7], $self->m[8]);
    return tmp;
  }
  Int32RMat() {
    struct Int32RMat *rm = (struct Int32RMat *) malloc(sizeof(struct Int32RMat));
    int32_rmat_identity(rm);
    return rm;
  }
  struct Int32RMat __mul__(struct Int32RMat *m_b2c) {
    struct Int32RMat m_a2c;
    int32_rmat_comp(&m_a2c, $self, m_b2c);
    return m_a2c;
  }
  struct Int32RMat __mul__(struct Int32Quat *q) {
    struct Int32RMat m_b2c;
    int32_rmat_of_quat(&m_b2c, q);
    struct Int32RMat m_a2c;
    int32_rmat_comp(&m_a2c, $self, &m_b2c);
    return m_a2c;
  }
  struct Int32Vect3 __mul__(struct Int32Vect3 *va) {
    struct Int32Vect3 v;
    int32_rmat_vmult(&v, $self, va);
    return v;
  }
  void set_identity() {
    int32_rmat_identity($self);
  }
  /*
  struct Int32RMat inverse() {
    struct Int32RMat inv;
    int32_rmat_inv(&inv, $self);
    return inv;
  }
  struct Int32RMat transposed() {
    struct Int32RMat inv;
    int32_rmat_inv(&inv, $self);
    return inv;
  }
  void invert() {
    int32_rmat_inv($self, $self);
  }
  void transpose() {
    int32_rmat_inv($self, $self);
  }
  */
  struct Int32Eulers to_eulers() {
    struct Int32Eulers e;
    int32_eulers_of_rmat(&e, $self);
    return e;
  }
  struct Int32Quat to_quat() {
    struct Int32Quat q;
    int32_quat_of_rmat(&q, $self);
    return q;
  }
};

%extend Int32Eulers {
  char *__str__() {
    static char tmp[1024];
    sprintf(tmp,"Eulers in deg (phi=%g, theta=%g, psi=%g)",
            DegOfRad(DOUBLE_OF_BFP($self->phi, INT32_ANGLE_FRAC)),
            DegOfRad(DOUBLE_OF_BFP($self->theta, INT32_ANGLE_FRAC)),
            DegOfRad(DOUBLE_OF_BFP($self->psi, INT32_ANGLE_FRAC)));
    return tmp;
  }
  Int32Eulers(int32_t phi=0, int32_t theta=0, int32_t psi=0) {
    struct Int32Eulers *v = (struct Int32Eulers *) malloc(sizeof(struct Int32Eulers));
    v->phi = phi;
    v->theta = theta;
    v->psi = psi;
    return v;
  }
  struct Int32Quat to_quat() {
    struct Int32Quat q;
    int32_quat_of_eulers(&q, $self);
    return q;
  }
  struct Int32RMat to_rmat() {
    struct Int32RMat r;
    int32_rmat_of_eulers(&r, $self);
    return r;
  }
};
