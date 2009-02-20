#include "pprz_geodetic_float.h"

#include "pprz_algebra_float.h"
#include <math.h>

void init_ltp_ref_from_ecef_f(struct LtpRef_f* ref_param, struct EcefCoor_f* ref_pos) {

  /* store ECEF0                 */
  FLOAT_VECT3_COPY(ref_param->ecef, *ref_pos);
  /* compute lla0                */
  lla_of_ecef_f(&ref_param->lla, &ref_param->ecef);
  /* store transformation matrix */
  const FLOAT_T sin_lat = sin(ref_param->lla.lat);
  const FLOAT_T cos_lat = cos(ref_param->lla.lat);
  const FLOAT_T sin_lon = sin(ref_param->lla.lon);
  const FLOAT_T cos_lon = cos(ref_param->lla.lon);
  ref_param->ltp_of_ecef.m[0] = -sin_lon;
  ref_param->ltp_of_ecef.m[1] =  cos_lon;
  ref_param->ltp_of_ecef.m[2] =  0.;
  ref_param->ltp_of_ecef.m[3] = -sin_lat*cos_lon;
  ref_param->ltp_of_ecef.m[4] = -sin_lat*sin_lon;
  ref_param->ltp_of_ecef.m[5] =  cos_lat;
  ref_param->ltp_of_ecef.m[6] =  cos_lat*cos_lon;
  ref_param->ltp_of_ecef.m[7] =  cos_lat*sin_lon;
  ref_param->ltp_of_ecef.m[8] =  sin_lat;

}

void init_ltp_ref_from_lla_f(struct LtpRef_f* ref_param, struct LlaCoor_f* ref_pos) {
  ref_param->lla.lon = ref_pos->lon;
  ref_param->lla.lat = ref_pos->lat;
  /* compute ecef */

}

void enu_of_ecef_f(struct EnuCoor_f* out, struct LtpRef_f* ref_param, struct EcefCoor_f* in) {

  struct EcefCoor_f delta;
  FLOAT_VECT3_DIFF(delta, *in, ref_param->ecef);
  FLOAT_MAT33_VECT3_MUL(*out, ref_param->ltp_of_ecef.m, delta);

}

void ned_of_ecef_f(struct LtpRef_f* ref_param, struct NedCoor_f* out, struct EcefCoor_f* in) {



}

/* http://en.wikipedia.org/wiki/Geodetic_system */
void lla_of_ecef_f(struct LlaCoor_f* out, struct EcefCoor_f* in) {

  // FIXME : make an ellipsoid struct
  static const FLOAT_T a = 6378137.0;         /* earth semimajor axis in meters */
  static const FLOAT_T f = 1./298.257223563;  /* reciprocal flattening */
  const FLOAT_T b = a*(1.-f);                 /* semi-minor axis */
  const FLOAT_T b2 = b*b;          
 
  const FLOAT_T e2 = 2.*f-(f*f);// first eccentricity squared
  const FLOAT_T ep2 = f*(2.-f)/((1.-f)*(1.-f)); // second eccentricity squared
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
  out->lat = atan( (in->z + ep2*zo)/r );
  out->lon = atan2(in->y,in->x);

}

