/* File : pprz_algebra_double.i */
%module algebra_double
%{
#include "math/pprz_algebra_double.h"
%}

/* don't wrap everything in header
%include "math/pprz_algebra_double.h"
* instead only wrap the structs and extend them with nicer to use methods
*/

struct DoubleVect2 {
  double x;
  double y;
};

struct DoubleVect3 {
  double x;
  double y;
  double z;
};

struct DoubleQuat {
  double qi;
  double qx;
  double qy;
  double qz;
};

struct DoubleRMat {
  double m[3 * 3];
};

struct DoubleEulers {
  double phi; ///< in radians
  double theta; ///< in radians
  double psi; ///< in radians
};

%extend DoubleVect2 {
  char *__str__() {
    static char tmp[1024];
    sprintf(tmp,"Vect2(%g, %g)", $self->x ,$self->y);
    return tmp;
  }
  DoubleVect2(double x=0.0, double y=0.0) {
    struct DoubleVect2 *v = (struct DoubleVect2 *) malloc(sizeof(struct DoubleVect2));
    v->x = x;
    v->y = y;
    return v;
  }
  struct DoubleVect2 __add__(struct DoubleVect2 *other) {
    struct DoubleVect2 v;
    v.x = $self->x + other->x;
    v.y = $self->y + other->y;
    return v;
  }
  struct DoubleVect2 __sub__(struct DoubleVect2 *other) {
    struct DoubleVect2 v;
    v.x = $self->x - other->x;
    v.y = $self->y - other->y;
    return v;
  }
  double norm() {
    return sqrt($self->x*$self->x + $self->y*$self->y);
  }
  void normalize() {
    const double n = sqrt($self->x*$self->x + $self->y*$self->y);
    if (n > 0) {
      $self->x /= n;
      $self->y /= n;
    }
  }
};

%extend DoubleVect3 {
  char *__str__() {
    static char tmp[1024];
    sprintf(tmp,"Vect3(%g, %g, %g)", $self->x ,$self->y, $self->z);
    return tmp;
  }
  DoubleVect3(double x=0.0, double y=0.0, double z=0.0) {
    struct DoubleVect3 *v = (struct DoubleVect3 *) malloc(sizeof(struct DoubleVect3));
    v->x = x;
    v->y = y;
    v->z = z;
    return v;
  }
  struct DoubleVect3 __add__(struct DoubleVect3 *other) {
    struct DoubleVect3 v;
    v.x = $self->x + other->x;
    v.y = $self->y + other->y;
    v.z = $self->z + other->z;
    return v;
  }
  struct DoubleVect3 __sub__(struct DoubleVect3 *other) {
    struct DoubleVect3 v;
    v.x = $self->x - other->x;
    v.y = $self->y - other->y;
    v.z = $self->z - other->z;
    return v;
  }
  struct DoubleVect3 __rmul__(struct DoubleQuat *q) {
    struct DoubleVect3 v;
    double_quat_vmult(&v, q, $self);
    return v;
  }
  double norm() {
    return double_vect3_norm($self);
  }
  void normalize() {
    double_vect3_normalize($self);
  }
};

%extend DoubleQuat {
  char *__str__() {
    static char tmp[1024];
    sprintf(tmp,"Quat(%g, %g, %g, %g)", $self->qi, $self->qx ,$self->qy, $self->qz);
    return tmp;
  }
  DoubleQuat(double qi=1.0, double qx=0.0, double qy=0.0, double qz=0.0) {
    struct DoubleQuat *v = (struct DoubleQuat *) malloc(sizeof(struct DoubleQuat));
    v->qi = qi;
    v->qx = qx;
    v->qy = qy;
    v->qz = qz;
    return v;
  }
  struct DoubleVect3 __mul__(struct DoubleVect3 *v) {
    struct DoubleVect3 vout;
    double_quat_vmult(&vout, $self, v);
    return vout;
  }
  double norm() {
    return double_quat_norm($self);
  }
  void normalize() {
    double_quat_normalize($self);
  }
  void set_identity() {
    double_quat_identity($self);
  }
  struct DoubleEulers to_eulers() {
    struct DoubleEulers e;
    double_eulers_of_quat(&e, $self);
    return e;
  }
};

%extend DoubleQuat {
  char *__str__() {
    static char tmp[1024];
    sprintf(tmp,"RMat[% .5f, % .5f, % .5f]\n    [% .5f, % .5f, % .5f]\n    [% .5f, % .5f, % .5f]",
            $self->m[0], $self->m[1], $self->m[2], $self->m[3], $self->m[4],
            $self->m[5], $self->m[6], $self->m[7], $self->m[8]);
    return tmp;
  }
  struct DoubleVect3 __mul__(struct DoubleVect3 *v) {
    struct DoubleVect3 vout;
    double_quat_vmult(&vout, $self, v);
    return vout;
  }
  double norm() {
    return double_quat_norm($self);
  }
  void normalize() {
    double_quat_normalize($self);
  }
  void set_identity() {
    double_quat_identity($self);
  }
  struct DoubleEulers to_eulers() {
    struct DoubleEulers e;
    double_eulers_of_quat(&e, $self);
    return e;
  }
};

%extend DoubleRMat {
  char *__str__() {
    static char tmp[1024];
    sprintf(tmp,"RMat[% .5f, % .5f, % .5f]\n    [% .5f, % .5f, % .5f]\n    [% .5f, % .5f, % .5f]",
            $self->m[0], $self->m[1], $self->m[2], $self->m[3], $self->m[4],
            $self->m[5], $self->m[6], $self->m[7], $self->m[8]);
    return tmp;
  }
  DoubleRMat() {
    struct DoubleRMat *rm = (struct DoubleRMat *) malloc(sizeof(struct DoubleRMat));
    double_rmat_identity(rm);
    return rm;
  }
  struct DoubleRMat __mul__(struct DoubleRMat *m_b2c) {
    struct DoubleRMat m_a2c;
    double_rmat_comp(&m_a2c, $self, m_b2c);
    return m_a2c;
  }
  struct DoubleRMat __mul__(struct DoubleQuat *q) {
    struct DoubleRMat m_b2c;
    double_rmat_of_quat(&m_b2c, q);
    struct DoubleRMat m_a2c;
    double_rmat_comp(&m_a2c, $self, &m_b2c);
    return m_a2c;
  }
  struct DoubleVect3 __mul__(struct DoubleVect3 *va) {
    struct DoubleVect3 v;
    double_rmat_vmult(&v, $self, va);
    return v;
  }
  void set_identity() {
    double_rmat_identity($self);
  }
  struct DoubleRMat inverse() {
    struct DoubleRMat inv;
    double_rmat_inv(&inv, $self);
    return inv;
  }
  struct DoubleRMat transposed() {
    struct DoubleRMat inv;
    double_rmat_inv(&inv, $self);
    return inv;
  }
};

%extend DoubleEulers {
  char *__str__() {
    static char tmp[1024];
    sprintf(tmp,"Eulers in deg (phi=%g, theta=%g, psi=%g)", DegOfRad($self->phi),
            DegOfRad($self->theta), DegOfRad($self->psi));
    return tmp;
  }
  DoubleEulers(double phi=0.0, double theta=0.0, double psi=0.0) {
    struct DoubleEulers *v = (struct DoubleEulers *) malloc(sizeof(struct DoubleEulers));
    v->phi = phi;
    v->theta = theta;
    v->psi = psi;
    return v;
  }
  struct DoubleQuat to_quat() {
    struct DoubleQuat q;
    double_quat_of_eulers(&q, $self);
    return q;
  }
  struct DoubleRMat to_rmat() {
    struct DoubleRMat r;
    double_rmat_of_eulers(&r, $self);
    return r;
  }
};
