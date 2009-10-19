#include "pprz_geodetic_double.h"

#include <math.h>

void ltp_def_from_ecef_d(struct LtpDef_d* def, struct EcefCoor_d* ecef) {

  /* store the origin of the tangeant plane       */
  VECT3_COPY(def->ecef, *ecef);
  /* compute the lla representation of the origin */
  lla_of_ecef_d(&def->lla, &def->ecef);
  /* store the rotation matrix                    */
  const double sin_lat = sin(def->lla.lat);
  const double cos_lat = cos(def->lla.lat);
  const double sin_lon = sin(def->lla.lon);
  const double cos_lon = cos(def->lla.lon);
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

/* http://en.wikipedia.org/wiki/Geodetic_system */
void lla_of_ecef_d(struct LlaCoor_d* lla, struct EcefCoor_d* ecef) {

  // FIXME : make an ellipsoid struct
  static const double a = 6378137.0;           /* earth semimajor axis in meters */
  static const double f = 1./298.257223563;    /* reciprocal flattening          */
  const double b = a*(1.-f);                   /* semi-minor axis                */
  const double b2 = b*b;                       
 
  const double e2 = 2.*f-(f*f);                /* first eccentricity squared     */
  const double ep2 = f*(2.-f)/((1.-f)*(1.-f)); /* second eccentricity squared    */
  const double E2 = a*a - b2;
 
  
  const double z2 = ecef->z*ecef->z;
  const double r2 = ecef->x*ecef->x+ecef->y*ecef->y;
  const double r = sqrt(r2);
  const double F = 54.*b2*z2;
  const double G = r2 + (1-e2)*z2 - e2*E2;
  const double c = (e2*e2*F*r2)/(G*G*G);
  const double s = powf( (1 + c + sqrt(c*c + 2*c)), 1./3.);
  const double s1 = 1+s+1/s;
  const double P = F/(3*s1*s1*G*G);
  const double Q = sqrt(1+2*e2*e2*P);
  const double ro = -(e2*P*r)/(1+Q) + sqrt((a*a/2)*(1+1/Q) - ((1-e2)*P*z2)/(Q*(1+Q)) - P*r2/2);
  const double tmp = (r - e2*ro)*(r - e2*ro);
  const double U = sqrt( tmp + z2 );
  const double V = sqrt( tmp + (1-e2)*z2 );
  const double zo = (b2*ecef->z)/(a*V);
 
  lla->alt = U*(1 - b2/(a*V));
  lla->lat = atan((ecef->z + ep2*zo)/r);
  lla->lon = atan2(ecef->y,ecef->x);

}

void ecef_of_lla_d(struct EcefCoor_d* ecef, struct LlaCoor_d* lla) {

  // FIXME : make an ellipsoid struct
  static const double a = 6378137.0;           /* earth semimajor axis in meters */
  static const double f = 1./298.257223563;    /* reciprocal flattening          */
  const double e2 = 2.*f-(f*f);                /* first eccentricity squared     */

  const double sin_lat = sinf(lla->lat);
  const double cos_lat = cosf(lla->lat);
  const double sin_lon = sinf(lla->lon);
  const double cos_lon = cosf(lla->lon);
  const double chi = sqrtf(1. - e2*sin_lat*sin_lat);
  const double a_chi = a / chi;

  ecef->x = (a_chi + lla->alt) * cos_lat * cos_lon;
  ecef->y = (a_chi + lla->alt) * cos_lat * sin_lon;
  ecef->z = (a_chi*(1. - e2) + lla->alt) * sin_lat;
}

void enu_of_ecef_point_d(struct EnuCoor_d* enu, struct LtpDef_d* def, struct EcefCoor_d* ecef) {
  struct EcefCoor_d delta;
  VECT3_DIFF(delta, *ecef, def->ecef);
  MAT33_VECT3_MUL(*enu, def->ltp_of_ecef, delta);
}

void ned_of_ecef_point_d(struct NedCoor_d* ned, struct LtpDef_d* def, struct EcefCoor_d* ecef) {
  struct EnuCoor_d enu;
  enu_of_ecef_point_d(&enu, def, ecef);
  ENU_OF_TO_NED(*ned, enu);
}

void enu_of_ecef_vect_d(struct EnuCoor_d* enu, struct LtpDef_d* def, struct EcefCoor_d* ecef) {
  MAT33_VECT3_MUL(*enu, def->ltp_of_ecef, *ecef);
}

void ned_of_ecef_vect_d(struct NedCoor_d* ned, struct LtpDef_d* def, struct EcefCoor_d* ecef) {
  struct EnuCoor_d enu;
  enu_of_ecef_vect_d(&enu, def, ecef);
  ENU_OF_TO_NED(*ned, enu);
}



void ecef_of_enu_point_d(struct EcefCoor_d* ecef, struct LtpDef_d* def, struct EnuCoor_d* enu) {
  MAT33_VECT3_TRANSP_MUL(*ecef, def->ltp_of_ecef, *enu);
  VECT3_ADD(*ecef, def->ecef);
}

void ecef_of_ned_point_d(struct EcefCoor_d* ecef, struct LtpDef_d* def, struct NedCoor_d* ned) {
  struct EnuCoor_d enu;
  ENU_OF_TO_NED(enu, *ned);
  ecef_of_enu_point_d(ecef, def, &enu);
}

void ecef_of_enu_vect_d(struct EcefCoor_d* ecef, struct LtpDef_d* def, struct EnuCoor_d* enu) {
  MAT33_VECT3_TRANSP_MUL(*ecef, def->ltp_of_ecef, *enu);
}

void ecef_of_ned_vect_d(struct EcefCoor_d* ecef, struct LtpDef_d* def, struct NedCoor_d* ned) {
  struct EnuCoor_d enu;
  ENU_OF_TO_NED(enu, *ned);
  ecef_of_enu_vect_d(ecef, def, &enu);
}





