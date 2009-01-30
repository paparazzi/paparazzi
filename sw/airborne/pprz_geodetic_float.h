#ifndef PPRZ_GEODETIC_FLOAT_H
#define PPRZ_GEODETIC_FLOAT_H

/* Earth Centered Earth Fixed in meters */
struct EcefCoor_f {
  FLOAT_T x;
  FLOAT_T y;
  FLOAT_T z;
};

/* lon, lat in radians */
/* alt in meters       */
struct LlaCoor_f {
  FLOAT_T lon;
  FLOAT_T lat;
  FLOAT_T alt;
};

/* North East Down local tangeant plane */
struct NedCoor_f {
  FLOAT_T x;
  FLOAT_T y;
  FLOAT_T z;
};

/* East North Down local tangeant plane */
struct EnuCoor_f {
  FLOAT_T x;
  FLOAT_T y;
  FLOAT_T z;
};

/* Local tangeant plane reference */
struct LtpRef_f {
  struct EcefCoor_f ecef;
  struct LlaCoor_f  lla;
};

extern void init_ltp_from_ecef_f(struct LtpRef_f* ref_param, struct EcefCoor_f* ref_pos);
extern void init_ltp_from_lla_f(struct LtpRef_f* ref_param, struct LlaCoor_f* ref_pos);
extern void enu_of_ecef_f(struct LtpRef_f* ref_param, struct EnuCoor_f* out, struct EcefCoor_f* in);

#endif /* PPRZ_GEODETIC_FLOAT_H */
