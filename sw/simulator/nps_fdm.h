#ifndef NPS_FDM
#define NPS_FDM


#include "std.h"
#include <matrix.h> 


struct NpsFdm {
  
  bool_t on_ground;
  
  VEC* ecef_pos;
  VEC* ecef_vel;
  VEC* ecef_accel;

  VEC* ltp_to_body_quat;
  VEC* ltp_body_rate;
  VEC* ltp_body_accel;

};

extern struct NpsFdm fdm;

extern void nps_fdm_init();
extern void nps_fdm_run_step(double* commands, double dt);

#endif /* NPS_FDM */
