/* File : pprz_geodetic_float.i */
%module geodetic_float
%{
#include "math/pprz_geodetic_float.h"
%}

%include "math/pprz_geodetic_float.h"

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
