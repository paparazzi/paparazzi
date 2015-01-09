/* File : pprz_geodetic_double.i */
%module geodetic_double
%{
#include "math/pprz_geodetic_double.h"
%}

%include "math/pprz_geodetic_double.h"


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
