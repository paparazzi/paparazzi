/* File : pprz_geodetic_float.i */
%module geodetic_float
%{
#include "math/pprz_geodetic_float.h"
%}

/* don't wrap everything in header
%include "math/pprz_geodetic_float.h"
* instead only wrap the structs and extend them with nicer to use methods
*/

struct EcefCoor_f {
  float x; ///< in meters
  float y; ///< in meters
  float z; ///< in meters
};

struct LlaCoor_f {
  float lat; ///< in radians
  float lon; ///< in radians
  float alt; ///< in meters above WGS84 reference ellipsoid
};

struct NedCoor_f {
  float x; ///< in meters
  float y; ///< in meters
  float z; ///< in meters
};

struct EnuCoor_f {
  float x; ///< in meters
  float y; ///< in meters
  float z; ///< in meters
};

struct UtmCoor_f {
  float north; ///< in meters
  float east; ///< in meters
  float alt; ///< in meters above WGS84 reference ellipsoid
  uint8_t zone; ///< UTM zone number
};

struct LtpDef_f {
  struct EcefCoor_f ecef; ///< origin of local frame in ECEF
  struct LlaCoor_f  lla; ///< origin of local frame in LLA
  struct FloatRMat ltp_of_ecef; ///< rotation from ECEF to local frame
  float hmsl; ///< Height above mean sea level in meters
};

%extend EcefCoor_f {
  char *__str__() {
    static char tmp[1024];
    sprintf(tmp,"ECEF(%.3f, %.3f, %.3f)", $self->x,$self->y,$self->z);
    return tmp;
  }
  EcefCoor_f(float x=0.0, float y=0.0, float z=0.0) {
    struct EcefCoor_f *v = (struct EcefCoor_f *) malloc(sizeof(struct EcefCoor_f));
    v->x = x;
    v->y = y;
    v->z = z;
    return v;
  }
  struct EcefCoor_f __add__(struct EcefCoor_f *other) {
    struct EcefCoor_f v;
    v.x = $self->x + other->x;
    v.y = $self->y + other->y;
    v.z = $self->z + other->z;
    return v;
  }
  struct EcefCoor_f __sub__(struct EcefCoor_f *other) {
    struct EcefCoor_f v;
    v.x = $self->x - other->x;
    v.y = $self->y - other->y;
    v.z = $self->z - other->z;
    return v;
  }
  struct LlaCoor_f to_lla() {
    struct LlaCoor_f lla;
    lla_of_ecef_f(&lla, $self);
    return lla;
  }
  struct LtpDef_f to_ltp_def() {
    struct LtpDef_f ltp;
    ltp_def_from_ecef_f(&ltp, $self);
    return ltp;
  }
  struct NedCoor_f to_ned(struct LtpDef_f *ltp) {
    struct NedCoor_f ned;
    ned_of_ecef_point_f(&ned, ltp, $self);
    return ned;
  }
  struct EnuCoor_f to_enu(struct LtpDef_f *ltp) {
    struct EnuCoor_f enu;
    enu_of_ecef_point_f(&enu, ltp, $self);
    return enu;
  }
  struct NedCoor_f to_ned_vect(struct LtpDef_f *ltp) {
    struct NedCoor_f ned;
    ned_of_ecef_vect_f(&ned, ltp, $self);
    return ned;
  }
  struct EnuCoor_f to_enu_vect(struct LtpDef_f *ltp) {
    struct EnuCoor_f enu;
    enu_of_ecef_vect_f(&enu, ltp, $self);
    return enu;
  }
};

%extend NedCoor_f {
  char *__str__() {
    static char tmp[1024];
    sprintf(tmp,"NED(% 3.3f,% 3.3f,% 3.3f)", $self->x,$self->y,$self->z);
    return tmp;
  }
  NedCoor_f(float x=0.0, float y=0.0, float z=0.0) {
    struct NedCoor_f *v = (struct NedCoor_f *) malloc(sizeof(struct NedCoor_f));
    v->x = x;
    v->y = y;
    v->z = z;
    return v;
  }
  struct NedCoor_f __add__(struct NedCoor_f *other) {
    struct NedCoor_f v;
    v.x = $self->x + other->x;
    v.y = $self->y + other->y;
    v.z = $self->z + other->z;
    return v;
  }
  struct NedCoor_f __sub__(struct NedCoor_f *other) {
    struct NedCoor_f v;
    v.x = $self->x - other->x;
    v.y = $self->y - other->y;
    v.z = $self->z - other->z;
    return v;
  }
  struct EcefCoor_f to_ecef(struct LtpDef_f *ltp) {
    struct EcefCoor_f ecef;
    ecef_of_ned_point_f(&ecef, ltp, $self);
    return ecef;
  }
  struct EcefCoor_f to_ecef_vect(struct LtpDef_f *ltp) {
    struct EcefCoor_f ecef;
    ecef_of_ned_vect_f(&ecef, ltp, $self);
    return ecef;
  }
};

%extend EnuCoor_f {
  char *__str__() {
    static char tmp[1024];
    sprintf(tmp,"ENU(% 3.3f,% 3.3f,% 3.3f)", $self->x,$self->y,$self->z);
    return tmp;
  }
  EnuCoor_f(float x=0.0, float y=0.0, float z=0.0) {
    struct EnuCoor_f *v = (struct EnuCoor_f *) malloc(sizeof(struct EnuCoor_f));
    v->x = x;
    v->y = y;
    v->z = z;
    return v;
  }
  struct EnuCoor_f __add__(struct EnuCoor_f *other) {
    struct EnuCoor_f v;
    v.x = $self->x + other->x;
    v.y = $self->y + other->y;
    v.z = $self->z + other->z;
    return v;
  }
  struct EnuCoor_f __sub__(struct EnuCoor_f *other) {
    struct EnuCoor_f v;
    v.x = $self->x - other->x;
    v.y = $self->y - other->y;
    v.z = $self->z - other->z;
    return v;
  }
  struct EcefCoor_f to_ecef(struct LtpDef_f *ltp) {
    struct EcefCoor_f ecef;
    ecef_of_enu_point_f(&ecef, ltp, $self);
    return ecef;
  }
  struct EcefCoor_f to_ecef_vect(struct LtpDef_f *ltp) {
    struct EcefCoor_f ecef;
    ecef_of_enu_vect_f(&ecef, ltp, $self);
    return ecef;
  }
};

%extend UtmCoor_f {
  char *__str__() {
    static char tmp[1024];
    sprintf(tmp,"UTM(north=%.3f, east=%.3f, alt=%g, zone=%d)", $self->north, $self->east, $self->alt, $self->zone);
    return tmp;
  }
  UtmCoor_f(float north=0.0, float east=0.0, float alt=0.0, uint8_t zone=0) {
    struct UtmCoor_f *v = (struct UtmCoor_f *) malloc(sizeof(struct UtmCoor_f));
    v->north = north;
    v->east = east;
    v->alt = alt;
    v->zone = zone;
    return v;
  }
  struct LlaCoor_f to_lla() {
    struct LlaCoor_f lla;
    lla_of_utm_f(&lla, $self);
    return lla;
  }
};

%extend LlaCoor_f {
  char *__str__() {
    static char tmp[1024];
    sprintf(tmp,"LLA deg (lat=%.7f, lon=%.7f, alt=%g)", DegOfRad($self->lat), DegOfRad($self->lon), $self->alt);
    return tmp;
  }
  LlaCoor_f(float lat=0.0, float lon=0.0, float alt=0.0) {
    struct LlaCoor_f *v = (struct LlaCoor_f *) malloc(sizeof(struct LlaCoor_f));
    v->lat = lat;
    v->lon = lon;
    v->alt = alt;
    return v;
  }
  struct EcefCoor_f to_ecef() {
    struct EcefCoor_f ecef;
    ecef_of_lla_f(&ecef, $self);
    return ecef;
  }
  struct NedCoor_f to_ned(struct LtpDef_f *ltp) {
    struct NedCoor_f ned;
    ned_of_lla_point_f(&ned, ltp, $self);
    return ned;
  }
  struct EnuCoor_f to_enu(struct LtpDef_f *ltp) {
    struct EnuCoor_f enu;
    enu_of_lla_point_f(&enu, ltp, $self);
    return enu;
  }
  struct UtmCoor_f to_utm() {
    struct UtmCoor_f utm;
    utm_of_lla_f(&utm, $self);
    return utm;
  }
  struct LtpDef_f to_ltp_def() {
    struct LtpDef_f ltp;
    ltp_def_from_lla_f(&ltp, $self);
    return ltp;
  }
};

%extend LtpDef_f {
  char *__str__() {
    static char tmp[1024];
    sprintf(tmp,"LtpDef: ECEF(%.3f, %.3f, %.3f), LLA(lat=%.7f, lon=%.7f, alt=%g), hmsl=%g",
            $self->ecef.x, $self->ecef.y, $self->ecef.z,
            DegOfRad($self->lla.lat), DegOfRad($self->lla.lon), $self->lla.alt, $self->hmsl);
    return tmp;
  }
};
