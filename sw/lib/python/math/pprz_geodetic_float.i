/* File : pprz_geodetic_float.i */
%module geodetic_float
%{
#include "math/pprz_geodetic_float.h"
%}

struct EcefCoor_f {
  float x; ///< in meters
  float y; ///< in meters
  float z; ///< in meters
};

/**
 * @brief vector in Latitude, Longitude and Altitude
 */
struct LlaCoor_f {
  float lat; ///< in radians
  float lon; ///< in radians
  float alt; ///< in meters above WGS84 reference ellipsoid
};

/**
 * @brief vector in North East Down coordinates
 * Units: meters */
struct NedCoor_f {
  float x; ///< in meters
  float y; ///< in meters
  float z; ///< in meters
};

/**
 * @brief vector in East North Up coordinates
 * Units: meters */
struct EnuCoor_f {
  float x; ///< in meters
  float y; ///< in meters
  float z; ///< in meters
};

/**
 * @brief position in UTM coordinates
 * Units: meters */
struct UtmCoor_f {
  float north; ///< in meters
  float east; ///< in meters
  float alt; ///< in meters above WGS84 reference ellipsoid
  uint8_t zone; ///< UTM zone number
};

/**
 * @brief definition of the local (flat earth) coordinate system
 * @details Defines the origin of the local coordinate system
 * in ECEF and LLA coordinates and the roation matrix from
 * ECEF to local frame */
struct LtpDef_f {
  struct EcefCoor_f ecef; ///< origin of local frame in ECEF
  struct LlaCoor_f  lla; ///< origin of local frame in LLA
  struct FloatRMat ltp_of_ecef; ///< rotation from ECEF to local frame
  float hmsl; ///< Height above mean sea level in meters
};

void lla_of_utm_f(struct LlaCoor_f *lla, struct UtmCoor_f *utm);
void utm_of_lla_f(struct UtmCoor_f *utm, struct LlaCoor_f *lla);
void ltp_def_from_ecef_f(struct LtpDef_f *def, struct EcefCoor_f *ecef);
void ltp_def_from_lla_f(struct LtpDef_f *def, struct LlaCoor_f *lla);
void lla_of_ecef_f(struct LlaCoor_f *out, struct EcefCoor_f *in);
void ecef_of_lla_f(struct EcefCoor_f *out, struct LlaCoor_f *in);
void enu_of_ecef_point_f(struct EnuCoor_f *enu, struct LtpDef_f *def, struct EcefCoor_f *ecef);
void ned_of_ecef_point_f(struct NedCoor_f *ned, struct LtpDef_f *def, struct EcefCoor_f *ecef);
void enu_of_ecef_vect_f(struct EnuCoor_f *enu, struct LtpDef_f *def, struct EcefCoor_f *ecef);
void ned_of_ecef_vect_f(struct NedCoor_f *ned, struct LtpDef_f *def, struct EcefCoor_f *ecef);
void enu_of_lla_point_f(struct EnuCoor_f *enu, struct LtpDef_f *def, struct LlaCoor_f *lla);
void ned_of_lla_point_f(struct NedCoor_f *ned, struct LtpDef_f *def, struct LlaCoor_f *lla);

/*  not enought precision with floats - used the double version */
void ecef_of_enu_point_f(struct EcefCoor_f *ecef, struct LtpDef_f *def, struct EnuCoor_f *enu);
void ecef_of_ned_point_f(struct EcefCoor_f *ecef, struct LtpDef_f *def, struct NedCoor_f *ned);
void ecef_of_enu_vect_f(struct EcefCoor_f *ecef, struct LtpDef_f *def, struct EnuCoor_f *enu);
void ecef_of_ned_vect_f(struct EcefCoor_f *ecef, struct LtpDef_f *def, struct NedCoor_f *ned);
/* end use double versions */
