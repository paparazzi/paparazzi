#ifndef NPS_FDM
#define NPS_FDM


#include "std.h"
#include <matrix.h> 


struct NpsFdm {
  
  bool_t on_ground;
  
  VEC* ecef_pos;
  VEC* ecef_vel;
  VEC* body_accel;

  VEC* ltp_to_body_quat;
  VEC* body_rate;
  VEC* body_rate_dot;

};

extern struct NpsFdm fdm;

extern void nps_fdm_init(double dt);
extern void nps_fdm_run_step(double* commands);

#endif /* NPS_FDM */
