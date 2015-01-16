/* File : pprz_geodetic_double.i */
%module geodetic_double
%{
#include "math/pprz_geodetic_double.h"
%}

/* don't wrap everything in header
%include "math/pprz_geodetic_double.h"
* instead only wrap the structs and extend them with nicer to use methods
*/

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

extern double gc_of_gd_lat_d(double gd_lat, double hmsl);

%extend EcefCoor_d {
  char *__str__() {
    static char tmp[1024];
    sprintf(tmp,"ECEF(%.3f, %.3f, %.3f)", $self->x,$self->y,$self->z);
    return tmp;
  }
  EcefCoor_d(double x=0.0, double y=0.0, double z=0.0) {
    struct EcefCoor_d *v = (struct EcefCoor_d *) malloc(sizeof(struct EcefCoor_d));
    v->x = x;
    v->y = y;
    v->z = z;
    return v;
  }
  struct EcefCoor_d __add__(struct EcefCoor_d *other) {
    struct EcefCoor_d v;
    v.x = $self->x + other->x;
    v.y = $self->y + other->y;
    v.z = $self->z + other->z;
    return v;
  }
  struct EcefCoor_d __sub__(struct EcefCoor_d *other) {
    struct EcefCoor_d v;
    v.x = $self->x - other->x;
    v.y = $self->y - other->y;
    v.z = $self->z - other->z;
    return v;
  }
  struct LlaCoor_d to_lla() {
    struct LlaCoor_d lla;
    lla_of_ecef_d(&lla, $self);
    return lla;
  }
  struct LtpDef_d to_ltp_def() {
    struct LtpDef_d ltp;
    ltp_def_from_ecef_d(&ltp, $self);
    return ltp;
  }
  struct NedCoor_d to_ned(struct LtpDef_d *ltp) {
    struct NedCoor_d ned;
    ned_of_ecef_point_d(&ned, ltp, $self);
    return ned;
  }
  struct EnuCoor_d to_enu(struct LtpDef_d *ltp) {
    struct EnuCoor_d enu;
    enu_of_ecef_point_d(&enu, ltp, $self);
    return enu;
  }
  struct NedCoor_d to_ned_vect(struct LtpDef_d *ltp) {
    struct NedCoor_d ned;
    ned_of_ecef_vect_d(&ned, ltp, $self);
    return ned;
  }
  struct EnuCoor_d to_enu_vect(struct LtpDef_d *ltp) {
    struct EnuCoor_d enu;
    enu_of_ecef_vect_d(&enu, ltp, $self);
    return enu;
  }
};

%extend NedCoor_d {
  char *__str__() {
    static char tmp[1024];
    sprintf(tmp,"NED(% 3.3f,% 3.3f,% 3.3f)", $self->x,$self->y,$self->z);
    return tmp;
  }
  NedCoor_d(double x=0.0, double y=0.0, double z=0.0) {
    struct NedCoor_d *v = (struct NedCoor_d *) malloc(sizeof(struct NedCoor_d));
    v->x = x;
    v->y = y;
    v->z = z;
    return v;
  }
  struct NedCoor_d __add__(struct NedCoor_d *other) {
    struct NedCoor_d v;
    v.x = $self->x + other->x;
    v.y = $self->y + other->y;
    v.z = $self->z + other->z;
    return v;
  }
  struct NedCoor_d __sub__(struct NedCoor_d *other) {
    struct NedCoor_d v;
    v.x = $self->x - other->x;
    v.y = $self->y - other->y;
    v.z = $self->z - other->z;
    return v;
  }
  struct EcefCoor_d to_ecef(struct LtpDef_d *ltp) {
    struct EcefCoor_d ecef;
    ecef_of_ned_point_d(&ecef, ltp, $self);
    return ecef;
  }
  struct EcefCoor_d to_ecef_vect(struct LtpDef_d *ltp) {
    struct EcefCoor_d ecef;
    ecef_of_ned_vect_d(&ecef, ltp, $self);
    return ecef;
  }
};

%extend EnuCoor_d {
  char *__str__() {
    static char tmp[1024];
    sprintf(tmp,"ENU(% 3.3f,% 3.3f,% 3.3f)", $self->x,$self->y,$self->z);
    return tmp;
  }
  EnuCoor_d(double x=0.0, double y=0.0, double z=0.0) {
    struct EnuCoor_d *v = (struct EnuCoor_d *) malloc(sizeof(struct EnuCoor_d));
    v->x = x;
    v->y = y;
    v->z = z;
    return v;
  }
  struct EnuCoor_d __add__(struct EnuCoor_d *other) {
    struct EnuCoor_d v;
    v.x = $self->x + other->x;
    v.y = $self->y + other->y;
    v.z = $self->z + other->z;
    return v;
  }
  struct EnuCoor_d __sub__(struct EnuCoor_d *other) {
    struct EnuCoor_d v;
    v.x = $self->x - other->x;
    v.y = $self->y - other->y;
    v.z = $self->z - other->z;
    return v;
  }
  struct EcefCoor_d to_ecef(struct LtpDef_d *ltp) {
    struct EcefCoor_d ecef;
    ecef_of_enu_point_d(&ecef, ltp, $self);
    return ecef;
  }
  struct EcefCoor_d to_ecef_vect(struct LtpDef_d *ltp) {
    struct EcefCoor_d ecef;
    ecef_of_enu_vect_d(&ecef, ltp, $self);
    return ecef;
  }
};

%extend UtmCoor_d {
  char *__str__() {
    static char tmp[1024];
    sprintf(tmp,"UTM(north=%.3f, east=%.3f, alt=%g, zone=%d)", $self->north, $self->east, $self->alt, $self->zone);
    return tmp;
  }
  UtmCoor_d(double north=0.0, double east=0.0, double alt=0.0, uint8_t zone=0) {
    struct UtmCoor_d *v = (struct UtmCoor_d *) malloc(sizeof(struct UtmCoor_d));
    v->north = north;
    v->east = east;
    v->alt = alt;
    v->zone = zone;
    return v;
  }
  struct LlaCoor_d to_lla() {
    struct LlaCoor_d lla;
    lla_of_utm_d(&lla, $self);
    return lla;
  }
};

%extend LlaCoor_d {
  char *__str__() {
    static char tmp[1024];
    sprintf(tmp,"LLA(lat=%.7f, lon=%.7f, alt=%g)", DegOfRad($self->lat), DegOfRad($self->lon), $self->alt);
    return tmp;
  }
  LlaCoor_d(double lat=0.0, double lon=0.0, double alt=0.0) {
    struct LlaCoor_d *v = (struct LlaCoor_d *) malloc(sizeof(struct LlaCoor_d));
    v->lat = lat;
    v->lon = lon;
    v->alt = alt;
    return v;
  }
  struct EcefCoor_d to_ecef() {
    struct EcefCoor_d ecef;
    ecef_of_lla_d(&ecef, $self);
    return ecef;
  }
  struct NedCoor_d to_ned(struct LtpDef_d *ltp) {
    struct NedCoor_d ned;
    ned_of_lla_point_d(&ned, ltp, $self);
    return ned;
  }
  struct EnuCoor_d to_enu(struct LtpDef_d *ltp) {
    struct EnuCoor_d enu;
    enu_of_lla_point_d(&enu, ltp, $self);
    return enu;
  }
};

%extend LtpDef_d {
  char *__str__() {
    static char tmp[1024];
    sprintf(tmp,"LtpDef: ECEF(%.3f, %.3f, %.3f), LLA(lat=%.7f, lon=%.7f, alt=%g), hmsl=%g",
            $self->ecef.x, $self->ecef.y, $self->ecef.z,
            DegOfRad($self->lla.lat), DegOfRad($self->lla.lon), $self->lla.alt, $self->hmsl);
    return tmp;
  }
};
