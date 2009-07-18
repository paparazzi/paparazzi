#ifndef PPRZ_GEODETIC_INT_H
#define PPRZ_GEODETIC_INT_H

#include "pprz_geodetic.h"

#include "std.h"
#include "pprz_algebra_int.h"

/* 
   Earth Centered Earth Fixed in centimeters 
*/
struct EcefCoor_i {
  int32_t x;
  int32_t y;
  int32_t z;
};

/* lon, lat in radians */
/* alt in meters       */
struct LlaCoor_i {
  int32_t lon;
  int32_t lat;
  int32_t alt;
};

/* North East Down local tangeant plane */
struct NedCoor_i {
  int32_t x;
  int32_t y;
  int32_t z;
};

/* East North Up local tangeant plane */
struct EnuCoor_i {
  int32_t x;
  int32_t y;
  int32_t z;
};

/* Local tangeant plane definition */
struct LtpDef_i {
  struct EcefCoor_i ecef;        /* Reference point in ecef */
  struct LlaCoor_i  lla;         /* Reference point in lla  */
  struct Int32Mat33 ltp_of_ecef; /* Rotation matrix         */
};

extern void ltp_def_from_ecef_i(struct LtpDef_i* def, struct EcefCoor_i* ecef);
//extern void ltp_def_from_lla_i(struct LtpRef_i* def, struct LlaCoor_i* lla);
extern void lla_of_ecef_i(struct LlaCoor_i* out, struct EcefCoor_i* in);
extern void ecef_of_lla_i(struct EcefCoor_i* out, struct LlaCoor_i* in);
extern void enu_of_ecef_point_i(struct EnuCoor_i* enu, struct LtpDef_i* def, struct EcefCoor_i* ecef);
extern void ned_of_ecef_point_i(struct NedCoor_i* ned, struct LtpDef_i* def, struct EcefCoor_i* ecef);
extern void enu_of_ecef_vect_i(struct EnuCoor_i* enu, struct LtpDef_i* def, struct EcefCoor_i* ecef);
extern void ned_of_ecef_vect_i(struct NedCoor_i* ned, struct LtpDef_i* def, struct EcefCoor_i* ecef);
extern void enu_of_lla_point_i(struct EnuCoor_i* enu, struct LtpDef_i* def, struct LlaCoor_i* lla);
extern void ned_of_lla_point_i(struct NedCoor_i* ned, struct LtpDef_i* def, struct LlaCoor_i* lla);

#define INT32_VECT3_ENU_OF_NED(_o, _i) {		\
    _o.x = _i.y;				\
    _o.y = _i.x;				\
    _o.z = -_i.z;				\
  }

#define INT32_VECT3_NED_OF_ENU(_o, _i) INT32_VECT3_ENU_OF_NED(_o,_i)

#define INT32_VECT2_ENU_OF_NED(_o, _i) {		\
    _o.x = _i.y;				\
    _o.y = _i.x;				\
  }

#define INT32_VECT2_NED_OF_ENU(_o, _i) INT32_VECT2_ENU_OF_NED(_o,_i)

#endif /* PPRZ_GEODETIC_INT_H */
