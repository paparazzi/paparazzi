#include "pprz_geodetic_float.h"

#include <math.h>

void init_ltp_from_ecef_f(struct LtpRef_f* ref_param, struct EcefCoor_f* ref_pos) {
  ref_param->ecef.x = ref_pos->x;
  ref_param->ecef.y = ref_pos->y;
  ref_param->ecef.z = ref_pos->z;
  /* compute lon and lat */

}

void init_ltp_from_lla_f(struct LtpRef_f* ref_param, struct LlaCoor_f* ref_pos) {
  ref_param->lla.lon = ref_pos->lon;
  ref_param->lla.lat = ref_pos->lat;
  /* compute ecef */

}

void enu_of_ecef_f(struct LtpRef_f* ref_param, struct EnuCoor_f* out, struct EcefCoor_f* in) {

  const FLOAT_T sin_lat = sin(ref_param->lla.lat);
  const FLOAT_T cos_lat = cos(ref_param->lla.lat);
  const FLOAT_T sin_lon = sin(ref_param->lla.lon);
  const FLOAT_T cos_lon = cos(ref_param->lla.lon);

  const FLOAT_T dx = in->x - ref_param->ecef.x;
  const FLOAT_T dy = in->y - ref_param->ecef.y;
  const FLOAT_T dz = in->z - ref_param->ecef.z;
  
  out->x = -sin_lon         * dx + cos_lon         * dy;
  out->y = -sin_lat*cos_lon * dx - sin_lat*sin_lon * dy + cos_lat * dz;
  out->z =  cos_lat*cos_lon * dx + cos_lat*sin_lon * dy + sin_lat * dz;

}
