#include "pprz_geodetic_float.h"

#include "pprz_algebra_float.h"
#include <math.h>

/* for ecef_of_XX functions the double versions are needed */
#include "pprz_geodetic_double.h"

void ltp_def_from_ecef_f(struct LtpDef_f* def, struct EcefCoor_f* ecef) {

  /* store the origin of the tangeant plane       */
  VECT3_COPY(def->ecef, *ecef);
  /* compute the lla representation of the origin */
  lla_of_ecef_f(&def->lla, &def->ecef);
  /* store the rotation matrix                    */
  const float sin_lat = sinf(def->lla.lat);
  const float cos_lat = cosf(def->lla.lat);
  const float sin_lon = sinf(def->lla.lon);
  const float cos_lon = cosf(def->lla.lon);
  def->ltp_of_ecef.m[0] = -sin_lon;
  def->ltp_of_ecef.m[1] =  cos_lon;
  def->ltp_of_ecef.m[2] =  0.; /* this element is always zero http://en.wikipedia.org/wiki/Geodetic_system#From_ECEF_to_ENU */
  def->ltp_of_ecef.m[3] = -sin_lat*cos_lon;
  def->ltp_of_ecef.m[4] = -sin_lat*sin_lon;
  def->ltp_of_ecef.m[5] =  cos_lat;
  def->ltp_of_ecef.m[6] =  cos_lat*cos_lon;
  def->ltp_of_ecef.m[7] =  cos_lat*sin_lon;
  def->ltp_of_ecef.m[8] =  sin_lat;

}

void ltp_def_from_lla_f(struct LtpDef_f* def, struct LlaCoor_f* lla) {
  /* store the origin of the tangeant plane */
  LLA_COPY(def->lla, *lla);
  /* compute the ecef representation of the origin */
  ecef_of_lla_f(&def->ecef, &def->lla);

  /* store the rotation matrix                    */
  const float sin_lat = sinf(def->lla.lat);
  const float cos_lat = cosf(def->lla.lat);
  const float sin_lon = sinf(def->lla.lon);
  const float cos_lon = cosf(def->lla.lon);

  def->ltp_of_ecef.m[0] = -sin_lon;
  def->ltp_of_ecef.m[1] =  cos_lon;
  def->ltp_of_ecef.m[2] =  0.; /* this element is always zero http://en.wikipedia.org/wiki/Geodetic_system#From_ECEF_to_ENU */
  def->ltp_of_ecef.m[3] = -sin_lat*cos_lon;
  def->ltp_of_ecef.m[4] = -sin_lat*sin_lon;
  def->ltp_of_ecef.m[5] =  cos_lat;
  def->ltp_of_ecef.m[6] =  cos_lat*cos_lon;
  def->ltp_of_ecef.m[7] =  cos_lat*sin_lon;
  def->ltp_of_ecef.m[8] =  sin_lat;
}

void enu_of_ecef_point_f(struct EnuCoor_f* enu, struct LtpDef_f* def, struct EcefCoor_f* ecef) {
  struct EcefCoor_f delta;
  VECT3_DIFF(delta, *ecef, def->ecef);
  MAT33_VECT3_MUL(*enu, def->ltp_of_ecef, delta);
}

void ned_of_ecef_point_f(struct NedCoor_f* ned, struct LtpDef_f* def, struct EcefCoor_f* ecef) {
  struct EnuCoor_f enu;
  enu_of_ecef_point_f(&enu, def, ecef);
  ENU_OF_TO_NED(*ned, enu);
}


void enu_of_ecef_vect_f(struct EnuCoor_f* enu, struct LtpDef_f* def, struct EcefCoor_f* ecef) {
  MAT33_VECT3_MUL(*enu, def->ltp_of_ecef, *ecef);
}

void ned_of_ecef_vect_f(struct NedCoor_f* ned, struct LtpDef_f* def, struct EcefCoor_f* ecef) {
  struct EnuCoor_f enu;
  enu_of_ecef_vect_f(&enu, def, ecef);
  ENU_OF_TO_NED(*ned, enu);
}

void enu_of_lla_point_f(struct EnuCoor_f* enu, struct LtpDef_f* def, struct LlaCoor_f* lla) {
  struct EcefCoor_f ecef;
  ecef_of_lla_f(&ecef,lla);
  enu_of_ecef_point_f(enu,def,&ecef);
}

void ned_of_lla_point_f(struct NedCoor_f* ned, struct LtpDef_f* def, struct LlaCoor_f* lla) {
  struct EcefCoor_f ecef;
  ecef_of_lla_f(&ecef,lla);
  ned_of_ecef_point_f(ned,def,&ecef);
}

/*
 * not enought precision with float - use double
 */
void ecef_of_enu_point_f(struct EcefCoor_f* ecef, struct LtpDef_f* def, struct EnuCoor_f* enu) {
  /* convert used floats to double */
  struct DoubleMat33 ltp_of_ecef_d;
  ltp_of_ecef_d.m[0] = (double) def->ltp_of_ecef.m[0];
  ltp_of_ecef_d.m[1] = (double) def->ltp_of_ecef.m[1];
  ltp_of_ecef_d.m[2] = (double) def->ltp_of_ecef.m[2];
  ltp_of_ecef_d.m[3] = (double) def->ltp_of_ecef.m[3];
  ltp_of_ecef_d.m[4] = (double) def->ltp_of_ecef.m[4];
  ltp_of_ecef_d.m[5] = (double) def->ltp_of_ecef.m[5];
  ltp_of_ecef_d.m[6] = (double) def->ltp_of_ecef.m[6];
  ltp_of_ecef_d.m[7] = (double) def->ltp_of_ecef.m[7];
  ltp_of_ecef_d.m[8] = (double) def->ltp_of_ecef.m[8];
  struct EnuCoor_f enu_d;
  enu_d.x = (double) enu->x;
  enu_d.y = (double) enu->y;
  enu_d.z = (double) enu->z;

  /* compute in double */
  struct EcefCoor_d ecef_d;
  MAT33_VECT3_TRANSP_MUL(ecef_d, ltp_of_ecef_d, enu_d);

  /* convert result back to float and add it*/
  ecef->x = (float) ecef_d.x + def->ecef.x;
  ecef->y = (float) ecef_d.y + def->ecef.y;
  ecef->z = (float) ecef_d.z + def->ecef.z;
}

void ecef_of_ned_point_f(struct EcefCoor_f* ecef, struct LtpDef_f* def, struct NedCoor_f* ned) {
  struct EnuCoor_f enu;
  ENU_OF_TO_NED(enu, *ned);
  ecef_of_enu_point_f(ecef, def, &enu);
}

void ecef_of_enu_vect_f(struct EcefCoor_f* ecef, struct LtpDef_f* def, struct EnuCoor_f* enu) {
  /* convert used floats to double */
  struct DoubleMat33 ltp_of_ecef_d;
  ltp_of_ecef_d.m[0] = (double) def->ltp_of_ecef.m[0];
  ltp_of_ecef_d.m[1] = (double) def->ltp_of_ecef.m[1];
  ltp_of_ecef_d.m[2] = (double) def->ltp_of_ecef.m[2];
  ltp_of_ecef_d.m[3] = (double) def->ltp_of_ecef.m[3];
  ltp_of_ecef_d.m[4] = (double) def->ltp_of_ecef.m[4];
  ltp_of_ecef_d.m[5] = (double) def->ltp_of_ecef.m[5];
  ltp_of_ecef_d.m[6] = (double) def->ltp_of_ecef.m[6];
  ltp_of_ecef_d.m[7] = (double) def->ltp_of_ecef.m[7];
  ltp_of_ecef_d.m[8] = (double) def->ltp_of_ecef.m[8];
  struct EnuCoor_f enu_d;
  enu_d.x = (double) enu->x;
  enu_d.y = (double) enu->y;
  enu_d.z = (double) enu->z;

  /* compute in double */
  struct EcefCoor_d ecef_d;
  MAT33_VECT3_TRANSP_MUL(ecef_d, ltp_of_ecef_d, enu_d);

  /* convert result back to float*/
  ecef->x = (float) ecef_d.x;
  ecef->y = (float) ecef_d.y;
  ecef->z = (float) ecef_d.z;
}

void ecef_of_ned_vect_f(struct EcefCoor_f* ecef, struct LtpDef_f* def, struct NedCoor_f* ned) {
  struct EnuCoor_f enu;
  ENU_OF_TO_NED(enu, *ned);
  ecef_of_enu_vect_f(ecef, def, &enu);
}
/* end use double versions */




/* http://en.wikipedia.org/wiki/Geodetic_system */
void lla_of_ecef_f(struct LlaCoor_f* out, struct EcefCoor_f* in) {

  // FIXME : make an ellipsoid struct
  static const float a = 6378137.0;           /* earth semimajor axis in meters */
  static const float f = 1./298.257223563;    /* reciprocal flattening          */
  const float b = a*(1.-f);                   /* semi-minor axis                */
  const float b2 = b*b;

  const float e2 = 2.*f-(f*f);                /* first eccentricity squared     */
  const float ep2 = f*(2.-f)/((1.-f)*(1.-f)); /* second eccentricity squared    */
  const float E2 = a*a - b2;


  const float z2 = in->z*in->z;
  const float r2 = in->x*in->x+in->y*in->y;
  const float r = sqrtf(r2);
  const float F = 54.*b2*z2;
  const float G = r2 + (1-e2)*z2 - e2*E2;
  const float c = (e2*e2*F*r2)/(G*G*G);
  const float s = powf( (1 + c + sqrtf(c*c + 2*c)), 1./3.);
  const float s1 = 1+s+1/s;
  const float P = F/(3*s1*s1*G*G);
  const float Q = sqrtf(1+2*e2*e2*P);
  const float ro = -(e2*P*r)/(1+Q) + sqrtf((a*a/2)*(1+1/Q) - ((1-e2)*P*z2)/(Q*(1+Q)) - P*r2/2);
  const float tmp = (r - e2*ro)*(r - e2*ro);
  const float U = sqrtf( tmp + z2 );
  const float V = sqrtf( tmp + (1-e2)*z2 );
  const float zo = (b2*in->z)/(a*V);

  out->alt = U*(1 - b2/(a*V));
  out->lat = atanf((in->z + ep2*zo)/r);
  out->lon = atan2f(in->y,in->x);

}

void ecef_of_lla_f(struct EcefCoor_f* out, struct LlaCoor_f* in) {

  // FIXME : make an ellipsoid struct
  static const float a = 6378137.0;           /* earth semimajor axis in meters */
  static const float f = 1./298.257223563;    /* reciprocal flattening          */
  const float e2 = 2.*f-(f*f);                /* first eccentricity squared     */

  const float sin_lat = sinf(in->lat);
  const float cos_lat = cosf(in->lat);
  const float sin_lon = sinf(in->lon);
  const float cos_lon = cosf(in->lon);
  const float chi = sqrtf(1. - e2*sin_lat*sin_lat);
  const float a_chi = a / chi;

  out->x = (a_chi + in->alt) * cos_lat * cos_lon;
  out->y = (a_chi + in->alt) * cos_lat * sin_lon;
  out->z = (a_chi*(1. - e2) + in->alt) * sin_lat;
}

