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

  struct NedCoor_d ltp_pos;
  struct NedCoor_d ltp_vel;

  struct DoubleQuat   ltp_to_body_quat;
  struct DoubleEulers ltp_to_body_eulers;
  struct DoubleRates body_rate;
  struct DoubleRates body_rate_dot;

  struct DoubleVect3 g_ltp;
  struct DoubleVect3 h_ltp;

};

extern struct NpsFdm fdm;

extern void nps_fdm_init(double dt);
extern void nps_fdm_run_step(double* commands);

#endif /* NPS_FDM */
