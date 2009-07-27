#ifndef NPS_FDM
#define NPS_FDM


#include "std.h"
#include "math/pprz_geodetic_double.h"
#include "math/pprz_algebra_double.h"

/*
  Notations for fdm variables
  ---------------------------
  coordinate system [ frame ] name

  ecef_inertial_vel is the time derivative of position
  with respect to inertial frame expressed in ECEF ( Earth Centered Earth Fixed)
  coordinate system.
*/

struct NpsFdm {
  
  double time;
  bool_t on_ground;

  /*  position */
  struct EcefCoor_d  ecef_pos;
  struct NedCoor_d ltpprz_pos;
  struct LlaCoor_d lla_pos;

  /*  velocity and acceleration wrt inertial frame expressed in ecef frame */
  //  struct EcefCoor_d  ecef_inertial_vel;
  //  struct EcefCoor_d  ecef_inertial_accel;
  /*  velocity and acceleration wrt ecef frame expressed in ecef frame     */
  struct EcefCoor_d  ecef_ecef_vel;
  struct EcefCoor_d  ecef_ecef_accel;
  /*  velocity and acceleration wrt ecef frame expressed in body frame     */
  struct DoubleVect3 body_ecef_vel;   /* aka UVW */
  struct DoubleVect3 body_ecef_accel; 
  /*  velocity and acceleration wrt ecef frame expressed in ltp frame     */
  struct NedCoor_d ltp_ecef_vel;
  struct NedCoor_d ltp_ecef_accel;
  /*  velocity and acceleration wrt ecef frame expressed in ltppprz frame */
  struct NedCoor_d ltpprz_ecef_vel;
  struct NedCoor_d ltpprz_ecef_accel;

  /* attitude */
  struct DoubleQuat   ltp_to_body_quat;
  struct DoubleEulers ltp_to_body_eulers;
  struct DoubleQuat   ltpprz_to_body_quat;
  struct DoubleEulers ltpprz_to_body_eulers;

  /*  velocity and acceleration wrt ecef frame expressed in body frame     */
  struct DoubleRates  body_ecef_rotvel;
  struct DoubleRates  body_ecef_rotaccel;

  struct DoubleVect3 ltp_g;
  struct DoubleVect3 ltp_h;

};

extern struct NpsFdm fdm;

extern void nps_fdm_init(double dt);
extern void nps_fdm_run_step(double* commands);

#endif /* NPS_FDM */
