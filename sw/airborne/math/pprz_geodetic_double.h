#ifndef PPRZ_GEODETIC_DOUBLE_H
#define PPRZ_GEODETIC_DOUBLE_H

#include <inttypes.h>

#include "pprz_geodetic.h"
#include "pprz_algebra_double.h"

/*
 * Definitions for coordinate systems
 * 
 */

/* Earth Centered Earth Fixed
 *   x, y, z in meters
 */
struct EcefCoor_d {
  double x;
  double y;
  double z;
};

/* Lon Lat Alt
 * lon, lat in radians, alt in meters
 */
struct LlaCoor_d {
  double lon;
  double lat;
  double alt;
};

/* North East Down local tangeant plane 
 *   x, y, z in meters
 */
struct NedCoor_d {
  double x;
  double y;
  double z;
};

/* East North Up local tangeant plane
 *   x, y, z in meters
 */
struct EnuCoor_d {
  double x;
  double y;
  double z;
};

/* Local tangeant plane reference */
struct LtpDef_d {
  struct EcefCoor_d  ecef;
  struct LlaCoor_d   lla;
  struct DoubleMat33 ltp_of_ecef;
};

/* UTM projection 
*/
struct UTMCoor_d {
  double east;
  double north;
  uint8_t zone;
};


/* 
 *  Convertions between coordinate systems 
 */
extern void ltp_def_from_ecef_d(struct LtpDef_d* def, struct EcefCoor_d* ecef);
extern void lla_of_ecef_d(struct LlaCoor_d* out, struct EcefCoor_d* in);
extern void ecef_of_lla_d(struct EcefCoor_d* out, struct LlaCoor_d* in);

extern void enu_of_ecef_point_d(struct EnuCoor_d* ned, struct LtpDef_d* def, struct EcefCoor_d* ecef);
extern void ned_of_ecef_point_d(struct NedCoor_d* ned, struct LtpDef_d* def, struct EcefCoor_d* ecef);

extern void enu_of_ecef_vect_d(struct EnuCoor_d* ned, struct LtpDef_d* def, struct EcefCoor_d* ecef);
extern void ned_of_ecef_vect_d(struct NedCoor_d* ned, struct LtpDef_d* def, struct EcefCoor_d* ecef);

extern void ecef_of_enu_point_d(struct EcefCoor_d* ecef, struct LtpDef_d* def, struct EnuCoor_d* enu);
extern void ecef_of_ned_point_d(struct EcefCoor_d* ecef, struct LtpDef_d* def, struct NedCoor_d* ned);

extern void ecef_of_enu_vect_d(struct EcefCoor_d* ecef, struct LtpDef_d* def, struct EnuCoor_d* enu);
extern void ecef_of_ned_vect_d(struct EcefCoor_d* ecef, struct LtpDef_d* def, struct NedCoor_d* ned);

extern double gc_of_gd_lat_d(double gd_lat, double hmsl);

extern void lla_of_utm(struct LlaCoor_d* out, struct UTMCoor_d* in);


#endif /* PPRZ_GEODETIC_DOUBLE_H */
