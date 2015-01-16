/* File : pprz_geodetic_int.i */
%module geodetic_int
%include "stdint.i"
%{
#include "math/pprz_geodetic_int.h"
%}

%include "math/pprz_geodetic_int.h"


%extend EcefCoor_i {
  char *__str__() {
    static char tmp[1024];
    sprintf(tmp,"ECEF(%d, %d, %d)", $self->x,$self->y,$self->z);
    return tmp;
  }
  EcefCoor_i(int32_t x=0, int32_t y=0, int32_t z=0) {
    struct EcefCoor_i *v = (struct EcefCoor_i *) malloc(sizeof(struct EcefCoor_i));
    v->x = x;
    v->y = y;
    v->z = z;
    return v;
  }
  struct EcefCoor_i __add__(struct EcefCoor_i *other) {
    struct EcefCoor_i v;
    v.x = $self->x + other->x;
    v.y = $self->y + other->y;
    v.z = $self->z + other->z;
    return v;
  }
  struct EcefCoor_i __sub__(struct EcefCoor_i *other) {
    struct EcefCoor_i v;
    v.x = $self->x - other->x;
    v.y = $self->y - other->y;
    v.z = $self->z - other->z;
    return v;
  }
};

%extend NedCoor_i {
  char *__str__() {
    static char tmp[1024];
    sprintf(tmp,"NED(%d, %d, %d)", $self->x,$self->y,$self->z);
    return tmp;
  }
  NedCoor_i(int32_t x=0, int32_t y=0, int32_t z=0) {
    struct NedCoor_i *v = (struct NedCoor_i *) malloc(sizeof(struct NedCoor_i));
    v->x = x;
    v->y = y;
    v->z = z;
    return v;
  }
  struct NedCoor_i __add__(struct NedCoor_i *other) {
    struct NedCoor_i v;
    v.x = $self->x + other->x;
    v.y = $self->y + other->y;
    v.z = $self->z + other->z;
    return v;
  }
  struct NedCoor_i __sub__(struct NedCoor_i *other) {
    struct NedCoor_i v;
    v.x = $self->x - other->x;
    v.y = $self->y - other->y;
    v.z = $self->z - other->z;
    return v;
  }
};

%extend EnuCoor_i {
  char *__str__() {
    static char tmp[1024];
    sprintf(tmp,"ENU(%d, %d, %d)", $self->x,$self->y,$self->z);
    return tmp;
  }
  EnuCoor_i(int32_t x=0, int32_t y=0, int32_t z=0) {
    struct EnuCoor_i *v = (struct EnuCoor_i *) malloc(sizeof(struct EnuCoor_i));
    v->x = x;
    v->y = y;
    v->z = z;
    return v;
  }
  struct EnuCoor_i __add__(struct EnuCoor_i *other) {
    struct EnuCoor_i v;
    v.x = $self->x + other->x;
    v.y = $self->y + other->y;
    v.z = $self->z + other->z;
    return v;
  }
  struct EnuCoor_i __sub__(struct EnuCoor_i *other) {
    struct EnuCoor_i v;
    v.x = $self->x - other->x;
    v.y = $self->y - other->y;
    v.z = $self->z - other->z;
    return v;
  }
};

%extend UtmCoor_i {
  char *__str__() {
    static char tmp[1024];
    sprintf(tmp,"UTM(north=%d, east=%d, alt=%d, zone=%d)", $self->north, $self->east, $self->alt, $self->zone);
    return tmp;
  }
  UtmCoor_i(int32_t north=0, int32_t east=0, int32_t alt=0, uint8_t zone=0) {
    struct UtmCoor_i *v = (struct UtmCoor_i *) malloc(sizeof(struct UtmCoor_i));
    v->north = north;
    v->east = east;
    v->alt = alt;
    v->zone = zone;
    return v;
  }
};

%extend LlaCoor_i {
  char *__str__() {
    static char tmp[1024];
    sprintf(tmp,"LLA(lat=%d, lon=%d, alt=%d)", $self->lat, $self->lon, $self->alt);
    return tmp;
  }
  LlaCoor_i(int32_t lat=0, int32_t lon=0, int32_t alt=0) {
    struct LlaCoor_i *v = (struct LlaCoor_i *) malloc(sizeof(struct LlaCoor_i));
    v->lat = lat;
    v->lon = lon;
    v->alt = alt;
    return v;
  }
};

%extend LtpDef_i {
  char *__str__() {
    static char tmp[1024];
    sprintf(tmp,"LtpDef: ECEF(%d, %d, %d), LLA(lat=%d, lon=%d, alt=%d), hmsl=%d",
            $self->ecef.x, $self->ecef.y, $self->ecef.z,
            $self->lla.lat, $self->lla.lon, $self->lla.alt, $self->hmsl);
    return tmp;
  }
};
