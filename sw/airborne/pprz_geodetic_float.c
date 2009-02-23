#include "pprz_geodetic_float.h"

#include "pprz_algebra_float.h"
#include <math.h>

void ltp_def_from_ecef_f(struct LtpDef_f* def, struct EcefCoor_f* ecef) {

  /* store the origin of the tangeant plane       */
  VECT3_COPY(def->ecef, *ecef);
  /* compute the lla representation of the origin */
  lla_of_ecef_f(&def->lla, &def->ecef);
  /* store the rotation matrix                    */
  const float sin_lat = sin(def->lla.lat);
  const float cos_lat = cos(def->lla.lat);
  const float sin_lon = sin(def->lla.lon);
  const float cos_lon = cos(def->lla.lon);
  def->ltp_of_ecef.m[0] = -sin_lon;
  def->ltp_of_ecef.m[1] =  cos_lon;
  def->ltp_of_ecef.m[2] =  0.;
  def->ltp_of_ecef.m[3] = -sin_lat*cos_lon;
  def->ltp_of_ecef.m[4] = -sin_lat*sin_lon;
  def->ltp_of_ecef.m[5] =  cos_lat;
  def->ltp_of_ecef.m[6] =  cos_lat*cos_lon;
  def->ltp_of_ecef.m[7] =  cos_lat*sin_lon;
  def->ltp_of_ecef.m[8] =  sin_lat;

}

#if 0
void init_ltp_ref_from_lla_f(struct LtpRef_f* def, struct LlaCoor_f* ref_pos) {
  def->lla.lon = ref_pos->lon;
  def->lla.lat = ref_pos->lat;
  /* compute ecef */

}
#endif

void enu_of_ecef_point_f(struct EnuCoor_f* enu, struct LtpDef_f* def, struct EcefCoor_f* ecef) {
  struct EcefCoor_f delta;
  VECT3_DIFF(delta, *ecef, def->ecef);
  MAT33_VECT3_MUL(*enu, def->ltp_of_ecef.m, delta);
}

void ned_of_ecef_point_f(struct NedCoor_f* ned, struct LtpDef_f* def, struct EcefCoor_f* ecef) {
  struct EnuCoor_f enu;
  enu_of_ecef_point_f(&enu, def, ecef);
  ENU_OF_TO_NED(*ned, enu);
}


void enu_of_ecef_vect_f(struct EnuCoor_f* enu, struct LtpDef_f* def, struct EcefCoor_f* ecef) {
  MAT33_VECT3_MUL(*enu, def->ltp_of_ecef.m, *ecef);
}

void ned_of_ecef_vect_f(struct NedCoor_f* ned, struct LtpDef_f* def, struct EcefCoor_f* ecef) {
  struct EnuCoor_f enu;
  enu_of_ecef_vect_f(&enu, def, ecef);
  ENU_OF_TO_NED(*ned, enu);
}

/* not enought precision with float - use double */
# if 0
void ecef_of_enu_point_f(struct EcefCoor_f* ecef, struct LtpDef_f* def, struct EnuCoor_f* enu) {
  MAT33_VECT3_TRANSP_MUL(*ecef, def->ltp_of_ecef.m, *enu);
  VECT3_ADD(*ecef, def->ecef);
}

void ecef_of_ned_point_f(struct EcefCoor_f* ecef, struct LtpDef_f* def, struct NedCoor_f* ned) {
  struct EnuCoor_f enu;
  ENU_OF_TO_NED(enu, *ned);
  ecef_of_enu_pos_f(ecef, def, &enu);
}

void ecef_of_enu_vect_f(struct EcefCoor_f* ecef, struct LtpDef_f* def, struct EnuCoor_f* enu) {
  MAT33_VECT3_TRANSP_MUL(*ecef, def->ltp_of_ecef.m, *enu);
}

void ecef_of_ned_vect_f(struct EcefCoor_f* ecef, struct LtpDef_f* def, struct NedCoor_f* ned) {
  struct EnuCoor_f enu;
  ENU_OF_TO_NED(enu, *ned);
  ecef_of_enu_vect_f(ecef, def, &enu);
}
#endif 




/* http://en.wikipedia.org/wiki/Geodetic_system */
void lla_of_ecef_f(struct LlaCoor_f* out, struct EcefCoor_f* in) {

  // FIXME : make an ellipsoid struct
  static const FLOAT_T a = 6378137.0;           /* earth semimajor axis in meters */
  static const FLOAT_T f = 1./298.257223563;    /* reciprocal flattening          */
  const FLOAT_T b = a*(1.-f);                   /* semi-minor axis                */
  const FLOAT_T b2 = b*b;                       
 
  const FLOAT_T e2 = 2.*f-(f*f);                /* first eccentricity squared     */
  const FLOAT_T ep2 = f*(2.-f)/((1.-f)*(1.-f)); /* second eccentricity squared    */
  const FLOAT_T E2 = a*a - b2;
 
  
  const FLOAT_T z2 = in->z*in->z;
  const FLOAT_T r2 = in->x*in->x+in->y*in->y;
  const FLOAT_T r = sqrt(r2);
  const FLOAT_T F = 54.*b2*z2;
  const FLOAT_T G = r2 + (1-e2)*z2 - e2*E2;
  const FLOAT_T c = (e2*e2*F*r2)/(G*G*G);
  const FLOAT_T s = powf( (1 + c + sqrt(c*c + 2*c)), 1./3.);
  const FLOAT_T s1 = 1+s+1/s;
  const FLOAT_T P = F/(3*s1*s1*G*G);
  const FLOAT_T Q = sqrt(1+2*e2*e2*P);
  const FLOAT_T ro = -(e2*P*r)/(1+Q) + sqrt((a*a/2)*(1+1/Q) - ((1-e2)*P*z2)/(Q*(1+Q)) - P*r2/2);
  const FLOAT_T tmp = (r - e2*ro)*(r - e2*ro);
  const FLOAT_T U = sqrt( tmp + z2 );
  const FLOAT_T V = sqrt( tmp + (1-e2)*z2 );
  const FLOAT_T zo = (b2*in->z)/(a*V);
 
  out->alt = U*(1 - b2/(a*V));
  out->lat = atan((in->z + ep2*zo)/r);
  out->lon = atan2(in->y,in->x);

}

