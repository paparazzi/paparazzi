/* File : pprz_geodetic_int.i */
%module geodetic_int
%include "stdint.i"
%{
#include "math/pprz_geodetic_int.h"
%}


/**
 * @brief vector in EarthCenteredEarthFixed coordinates
 * @details Origin at center of mass of the Earth. Z-axis is pointing north,
 * the x-axis intersects the sphere of the earth at 0° latitude (Equator)
 * and 0° longitude (Greenwich). Y-axis completes it to right-hand system.
 * Units: centimeters */
struct EcefCoor_i {
  int32_t x; ///< in centimeters
  int32_t y; ///< in centimeters
  int32_t z; ///< in centimeters
};

/**
 * @brief vector in Latitude, Longitude and Altitude
 */
struct LlaCoor_i {
  int32_t lat; ///< in degrees*1e7
  int32_t lon; ///< in degrees*1e7
  int32_t alt; ///< in millimeters above WGS84 reference ellipsoid
};

/**
 * @brief vector in North East Down coordinates
 */
struct NedCoor_i {
  int32_t x;  ///< North
  int32_t y;  ///< East
  int32_t z;  ///< Down
};

/**
 * @brief vector in East North Up coordinates
 */
struct EnuCoor_i {
  int32_t x;  ///< East
  int32_t y;  ///< North
  int32_t z;  ///< Up
};

/**
 * @brief position in UTM coordinates
 */
struct UtmCoor_i {
  int32_t north; ///< in centimeters
  int32_t east; ///< in centimeters
  int32_t alt; ///< in millimeters above WGS84 reference ellipsoid
  uint8_t zone; ///< UTM zone number
};

/**
 * @brief definition of the local (flat earth) coordinate system
 * @details Defines the origin of the local coordinate system
 * in ECEF and LLA coordinates and the roation matrix from
 * ECEF to local frame */
struct LtpDef_i {
  struct EcefCoor_i ecef;        ///< Reference point in ecef
  struct LlaCoor_i  lla;         ///< Reference point in lla
  struct Int32RMat ltp_of_ecef; ///< Rotation matrix
  int32_t hmsl;                  ///< Height above mean sea level in mm
};

extern void ltp_of_ecef_rmat_from_lla_i(struct Int32RMat *ltp_of_ecef, struct LlaCoor_i *lla);
extern void ltp_def_from_ecef_i(struct LtpDef_i *def, struct EcefCoor_i *ecef);
extern void ltp_def_from_lla_i(struct LtpDef_i *def, struct LlaCoor_i *lla);
extern void lla_of_ecef_i(struct LlaCoor_i *out, struct EcefCoor_i *in);
extern void ecef_of_lla_i(struct EcefCoor_i *out, struct LlaCoor_i *in);
extern void enu_of_ecef_point_i(struct EnuCoor_i *enu, struct LtpDef_i *def, struct EcefCoor_i *ecef);
extern void ned_of_ecef_point_i(struct NedCoor_i *ned, struct LtpDef_i *def, struct EcefCoor_i *ecef);
extern void enu_of_ecef_pos_i(struct EnuCoor_i *enu, struct LtpDef_i *def, struct EcefCoor_i *ecef);
extern void ned_of_ecef_pos_i(struct NedCoor_i *ned, struct LtpDef_i *def, struct EcefCoor_i *ecef);
extern void enu_of_ecef_vect_i(struct EnuCoor_i *enu, struct LtpDef_i *def, struct EcefCoor_i *ecef);
extern void ned_of_ecef_vect_i(struct NedCoor_i *ned, struct LtpDef_i *def, struct EcefCoor_i *ecef);
extern void enu_of_lla_point_i(struct EnuCoor_i *enu, struct LtpDef_i *def, struct LlaCoor_i *lla);
extern void ned_of_lla_point_i(struct NedCoor_i *ned, struct LtpDef_i *def, struct LlaCoor_i *lla);
extern void enu_of_lla_vect_i(struct EnuCoor_i *enu, struct LtpDef_i *def, struct LlaCoor_i *lla);
extern void ned_of_lla_vect_i(struct NedCoor_i *ned, struct LtpDef_i *def, struct LlaCoor_i *lla);
extern void ecef_of_enu_point_i(struct EcefCoor_i *ecef, struct LtpDef_i *def, struct EnuCoor_i *enu);
extern void ecef_of_ned_point_i(struct EcefCoor_i *ecef, struct LtpDef_i *def, struct NedCoor_i *ned);
extern void ecef_of_enu_pos_i(struct EcefCoor_i *ecef, struct LtpDef_i *def, struct EnuCoor_i *enu);
extern void ecef_of_ned_pos_i(struct EcefCoor_i *ecef, struct LtpDef_i *def, struct NedCoor_i *ned);
extern void ecef_of_enu_vect_i(struct EcefCoor_i *ecef, struct LtpDef_i *def, struct EnuCoor_i *enu);
extern void ecef_of_ned_vect_i(struct EcefCoor_i *ecef, struct LtpDef_i *def, struct NedCoor_i *ned);
