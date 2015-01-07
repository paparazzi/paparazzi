/* File : pprz_geodetic_double.i */
%module geodetic_double
%{
#include "math/pprz_geodetic_double.h"
%}

struct EcefCoor_d {
  double x; ///< in meters
  double y; ///< in meters
  double z; ///< in meters
};

struct LlaCoor_d {
  double lat; ///< in radians
  double lon; ///< in radians
  double alt; ///< in meters above WGS84 reference ellipsoid
};

struct NedCoor_d {
  double x; ///< in meters
  double y; ///< in meters
  double z; ///< in meters
};

struct EnuCoor_d {
  double x; ///< in meters
  double y; ///< in meters
  double z; ///< in meters
};

struct UtmCoor_d {
  double north; ///< in meters
  double east; ///< in meters
  double alt; ///< in meters above WGS84 reference ellipsoid
  uint8_t zone; ///< UTM zone number
};

struct LtpDef_d {
  struct EcefCoor_d  ecef; ///< origin of local frame in ECEF
  struct LlaCoor_d   lla; ///< origin of local frame in LLA
  struct DoubleRMat ltp_of_ecef; ///< rotation from ECEF to local frame
  double hmsl; ///< height in meters above mean sea level
};

void lla_of_utm_d(struct LlaCoor_d *out, struct UtmCoor_d *in);
void ltp_def_from_ecef_d(struct LtpDef_d *def, struct EcefCoor_d *ecef);
void lla_of_ecef_d(struct LlaCoor_d *out, struct EcefCoor_d *in);
void ecef_of_lla_d(struct EcefCoor_d *out, struct LlaCoor_d *in);

void enu_of_ecef_point_d(struct EnuCoor_d *ned, struct LtpDef_d *def, struct EcefCoor_d *ecef);
void ned_of_ecef_point_d(struct NedCoor_d *ned, struct LtpDef_d *def, struct EcefCoor_d *ecef);

void enu_of_ecef_vect_d(struct EnuCoor_d *ned, struct LtpDef_d *def, struct EcefCoor_d *ecef);
void ned_of_ecef_vect_d(struct NedCoor_d *ned, struct LtpDef_d *def, struct EcefCoor_d *ecef);

void ecef_of_enu_point_d(struct EcefCoor_d *ecef, struct LtpDef_d *def, struct EnuCoor_d *enu);
void ecef_of_ned_point_d(struct EcefCoor_d *ecef, struct LtpDef_d *def, struct NedCoor_d *ned);

void ecef_of_enu_vect_d(struct EcefCoor_d *ecef, struct LtpDef_d *def, struct EnuCoor_d *enu);
void ecef_of_ned_vect_d(struct EcefCoor_d *ecef, struct LtpDef_d *def, struct NedCoor_d *ned);

void enu_of_lla_point_d(struct EnuCoor_d *enu, struct LtpDef_d *def, struct LlaCoor_d *lla);
void ned_of_lla_point_d(struct NedCoor_d *ned, struct LtpDef_d *def, struct LlaCoor_d *lla);

double gc_of_gd_lat_d(double gd_lat, double hmsl);
