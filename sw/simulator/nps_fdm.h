#ifndef NPS_FDM
#define NPS_FDM


#include "std.h"
#include "pprz_geodetic_double.h"
#include "pprz_algebra_double.h"


struct NpsFdm {
  
  bool_t on_ground;

  struct EcefCoor_d  ecef_pos;
  struct EcefCoor_d  ecef_vel;
  struct DoubleVect3 body_accel;

  struct DoubleQuat  ltp_to_body_quat;
  struct DoubleRates body_rate;
  struct DoubleRates body_rate_dot;

};

extern struct NpsFdm fdm;

extern void nps_fdm_init(double dt);
extern void nps_fdm_run_step(double* commands);

#endif /* NPS_FDM */
