// toytronics_setpoint.c
// Greg Horn, Joby Robotics 2011

#include "toytronics_setpoint.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

#include "toytronics_types.h"

#include "generated/airframe.h" // for aircraft constants

#include "math/pprz_algebra_int.h" // Int32Quat etc types
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_int.h" // for Int32Quat stab_att_sp_quat
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_int.h" // for RC_UPDATE_FREQ
#include "firmwares/rotorcraft/guidance/guidance_h.h" // for mode #defines
#include "subsystems/ahrs.h" // for ahrs_float

#include "subsystems/radio_control.h" // for rc

setpoint_t setpoint, setpoint_old, stashed_setpoint;

xyz_t setpoint_incremental_bounds_deg = { SETPOINT_MODE_2_BOUND_QUAT_DEG_X,
                                          SETPOINT_MODE_2_BOUND_QUAT_DEG_Y,
                                          SETPOINT_MODE_2_BOUND_QUAT_DEG_Z};

double setpoint_absolute_heading_bound_deg = SETPOINT_BOUND_ERROR_HEADING_DEG;
double hover_pitch_trim_deg = SETPOINT_HOVER_PITCH_TRIM_DEG;

xyz_t r_n2h_n = {0,0,0}; // local NED to hover position setpoint vector
xyz_t r_b2h_n = {0,0,0}; // body to hover setpoint vector
double hover_throttle0;
double hover_x_integrated_error = 0;
double hover_y_integrated_error = 0;

// For hover:
// You would do heading with the euler "roll" but this doesn't work
// because of the singularity. Therefore we're just using the generic
// "heading" variable.

double roll_to_yaw_rate_ff_factor = SETPOINT_ROLL_TO_YAW_RATE_FF_FACTOR;
double smooth_transition_angle = 0.0;

#define RC_INCREMENTAL_DEADBAND 0.02

/**************** prototypes *******************/
static void setpoint_smooth_transition_reset(void);

static void toytronics_sp_enter_incremental(void);
static void toytronics_sp_enter_absolute_hover(void);
static void toytronics_sp_enter_absolute_forward(void);

static void toytronics_sp_set_absolute_heading_bound_deg(double new_bound);
static void toytronics_sp_set_incremental_bounds_deg(double bound_x, double bound_y, double bound_z);

// pprz interface
static const quat_t * get_q_n2b(void);
static const euler_t * get_e_n2b(void);
static const rc_t * get_rc(void);
static void set_paparazzi_setpoint(void);

static void inner_set_forward_gains(void);
static void inner_set_hover_gains(void);
static void inner_set_aerobatic_gains(void);
static void inner_setpoint_reset(void);

/**************  paparazzi interface ************/
static void inner_set_forward_gains() {};   //dummy
static void inner_set_hover_gains() {};     //dummy
static void inner_set_aerobatic_gains() {}; //dummy
static void inner_setpoint_reset() {}; //dummy

static const quat_t * get_q_n2b(void){
  static quat_t q_n2b = {1,0,0,0};
  /* q_n2b.q0 = ahrs_float.ltp_to_body_quat.qi; */
  /* q_n2b.q1 = ahrs_float.ltp_to_body_quat.qx; */
  /* q_n2b.q2 = ahrs_float.ltp_to_body_quat.qy; */
  /* q_n2b.q3 = ahrs_float.ltp_to_body_quat.qz; */
  q_n2b.q0 = QUAT1_FLOAT_OF_BFP(ahrs.ltp_to_body_quat.qi);
  q_n2b.q1 = QUAT1_FLOAT_OF_BFP(ahrs.ltp_to_body_quat.qx);
  q_n2b.q2 = QUAT1_FLOAT_OF_BFP(ahrs.ltp_to_body_quat.qy);
  q_n2b.q3 = QUAT1_FLOAT_OF_BFP(ahrs.ltp_to_body_quat.qz);
  return &q_n2b;
}

static const euler_t * get_e_n2b(void){
  static euler_t e_n2b = {0,0,0};
  /* e_n2b.roll  = ahrs_float.ltp_to_body_euler.phi; */
  /* e_n2b.pitch = ahrs_float.ltp_to_body_euler.theta; */
  /* e_n2b.yaw   = ahrs_float.ltp_to_body_euler.psi; */
  e_n2b.roll  = ANGLE_FLOAT_OF_BFP(ahrs.ltp_to_body_euler.phi);
  e_n2b.pitch = ANGLE_FLOAT_OF_BFP(ahrs.ltp_to_body_euler.theta);
  e_n2b.yaw   = ANGLE_FLOAT_OF_BFP(ahrs.ltp_to_body_euler.psi);
  return &e_n2b;
}

static const rc_t * get_rc(void){
  static rc_t rc = {0,0,0,0};

  rc.roll  = ((double)radio_control.values[RADIO_ROLL])/MAX_PPRZ;
  rc.pitch = ((double)radio_control.values[RADIO_PITCH])/MAX_PPRZ;
  rc.yaw   = ((double)radio_control.values[RADIO_YAW])/MAX_PPRZ;

  return &rc;
}


static void
set_paparazzi_setpoint(void)
{
  stab_att_sp_quat.qi = QUAT1_BFP_OF_REAL(setpoint.q_n2sp.q0);
  stab_att_sp_quat.qx = QUAT1_BFP_OF_REAL(setpoint.q_n2sp.q1);
  stab_att_sp_quat.qy = QUAT1_BFP_OF_REAL(setpoint.q_n2sp.q2);
  stab_att_sp_quat.qz = QUAT1_BFP_OF_REAL(setpoint.q_n2sp.q3);
}



/****************** math/helper functions ******************/

// wrap to (-pi,pi)
static void
wrap_to_pi(double * angle){
  while (*angle > M_PI)
    *angle -= 2*M_PI;
  while (*angle < -M_PI)
    *angle += 2*M_PI;
}

// get "heading" from attitude quat q_n2b:
// first find the shortest path to vertical (q_n2h),
// then find how much rotation q_n2h has relative to north

static double
get_heading_from_q_n2b(const quat_t * const q_n2b)
{
  double R13 = 2*( -q_n2b->q0*q_n2b->q2 + q_n2b->q1*q_n2b->q3 );
  double R23 = 2*(  q_n2b->q0*q_n2b->q1 + q_n2b->q2*q_n2b->q3 );
  double R33 = -1 + 2*q_n2b->q0*q_n2b->q0 + 2*q_n2b->q3*q_n2b->q3;

  // get body to heading quat q_n2h
  if (fabs(R13) >= 1.0)
    R13 /= R13 + 1.0e-12;
  double theta = acos(-R13);
  
  double norm_rb = sqrt( R23*R23 + R33*R33 ) + 1e-12;
  quat_t q_b2h = {              cos(0.5*theta), 
                                           0.0, 
                    R33/norm_rb*sin(0.5*theta),
                   -R23/norm_rb*sin(0.5*theta)};

  quat_t q_n2h;
  quat_mult( &q_n2h, q_n2b, &q_b2h );
  
  quat_t q_y90 = {sqrt(2.0)/2.0, 0.0, sqrt(2.0)/2.0, 0.0};
  
  quat_t q_theta;
  quat_inv_mult( &q_theta, &q_y90, &q_n2h );

  if ( fabs(q_theta.q0) > 1.0)
    q_theta.q0 /= q_theta.q0 + 1.0e-12;

  double heading = -2*acos(q_theta.q0);
  if (q_theta.q1 < 0.0)
    heading = -heading;

  return heading;
}


// apply deadband to rc signal, signal is assumed to range from -1 to 1
static double
apply_deadband( double rc, double deadband)
{
  if (fabs(rc) < deadband)
    return 0;
  if (rc > 0)
    return rc - deadband;
  else
    return rc + deadband;
}

// if pitch went from -90 to 180 degrees
static double
get_full_range_pitch(const quat_t * const q_n2b, const euler_t * const e_n2b)
{
  double R[9];
  dcm_of_quat_a2b( R, q_n2b );
  if (R[8] > 0)
    return e_n2b->pitch;
  else
    return acos( R[8]/cos(atan( R[5]/R[8] )) );
}

static double
get_fudged_yaw(const quat_t * const q_n2b, const euler_t * const e_n2b)
{
  // first get a pitch that goes from -90 to 180 degrees
  double extend_o_pitch = get_full_range_pitch(q_n2b, e_n2b);

  // now bound the aircraft's full-range pitch
  double pitch_error = extend_o_pitch - 60*M_PI/180.0;
  if (pitch_error > 0){
    quat_t q_fudgetown = {cos(0.5*pitch_error), 0, -sin(0.5*pitch_error), 0};
    q_fudgetown = quat_mult_ret( *q_n2b, q_fudgetown );
    euler_t e_fudgetown;
    euler321_of_quat( &e_fudgetown, &q_fudgetown );
    return e_fudgetown.yaw;
  } else
    return e_n2b->yaw;
}

/****************** enter mode functions ******************/
static void
setpoint_smooth_transition_reset()
{
  const quat_t * q_n2b = get_q_n2b();
  const euler_t * e_n2b = get_e_n2b();
  smooth_transition_angle = get_full_range_pitch(q_n2b, e_n2b);
}

static void
toytronics_sp_enter_incremental()
{
  const quat_t * q_n2b = get_q_n2b();

  // copy current attitude to setpoint attitude
  memcpy( &(setpoint.q_n2sp), q_n2b, sizeof(quat_t));
  setpoint_old = setpoint;
  stashed_setpoint = setpoint;
}

static void
toytronics_sp_enter_absolute_hover()
{
  const quat_t * q_n2b = get_q_n2b();

  // copy current heading to setpoint heading
  setpoint.setpoint_heading = get_heading_from_q_n2b(q_n2b);
  memcpy( &(setpoint.q_n2sp), q_n2b, sizeof(quat_t));
  setpoint_old = setpoint;
}

static void
toytronics_sp_enter_absolute_forward()
{
  const quat_t * q_n2b = get_q_n2b();
  const euler_t * e_n2b = get_e_n2b();

  // get current yaw
  // fudge to deal with singularity for now
  setpoint.setpoint_heading = get_fudged_yaw(q_n2b, e_n2b);
  memcpy( &(setpoint.q_n2sp), q_n2b, sizeof(quat_t));
  setpoint_old = setpoint;
}

static void
toytronics_sp_set_absolute_heading_bound_deg(double new_bound){
  setpoint_absolute_heading_bound_deg = new_bound;
}

static void
toytronics_sp_set_incremental_bounds_deg(double bound_x, double bound_y, double bound_z)
{
  setpoint_incremental_bounds_deg.x = bound_x;
  setpoint_incremental_bounds_deg.y = bound_y;
  setpoint_incremental_bounds_deg.z = bound_z;
}


/****************** core functions ******************/
void
toytronics_set_sp_absolute_hover_from_rc()
{
  double dt = 1.0/RC_UPDATE_FREQ;
  const rc_t * const rc = get_rc();
  const quat_t * const q_n2b = get_q_n2b();

  setpoint_old = setpoint;
  
#ifdef SWAP_STICKS_FOR_SCOTT
  // local copies to allow implementing a deadband
  double rcp = rc->pitch;
  double rcy = rc->yaw;
  double rcr = -apply_deadband(rc->roll, SETPOINT_DEADBAND);
#else
  // local copies to allow implementing a deadband
  double rcp = rc->pitch;
  double rcy = rc->roll;
  double rcr = apply_deadband(rc->yaw, SETPOINT_DEADBAND);
#endif  

  // integrate stick to get setpoint heading
  setpoint.setpoint_heading += dt*SETPOINT_MAX_STICK_DEG_PER_SEC*M_PI/180.0*rcr;
  wrap_to_pi(&(setpoint.setpoint_heading));

  // set pitch/yaw from stick based on current body roll
  double pitch_body = (rcp * SETPOINT_MAX_STICK_ANGLE_DEG + hover_pitch_trim_deg)*M_PI/180.0;
  double yaw_body   = rcy * SETPOINT_MAX_STICK_ANGLE_DEG*M_PI/180.0;

  // rotate pitch/yaw commands into setpoint frame
  setpoint.estimated_heading = get_heading_from_q_n2b(q_n2b);
  double error_heading = setpoint.setpoint_heading - setpoint.estimated_heading;
  wrap_to_pi(&error_heading);

  // don't let setpoint drift too far
  BOUND(error_heading, -setpoint_absolute_heading_bound_deg*M_PI/180.0, setpoint_absolute_heading_bound_deg*M_PI/180.0);
  setpoint.setpoint_heading = setpoint.estimated_heading + error_heading;  
  
  double pitch_setpoint =  pitch_body*cos(error_heading) - yaw_body*sin(error_heading);
  double yaw_setpoint   =  pitch_body*sin(error_heading) + yaw_body*cos(error_heading);

  // generate the setpoint quaternion
  // start straight up

  setpoint.q_n2sp.q0 = sqrt(2.0)/2.0;
  setpoint.q_n2sp.q1 = 0.0;
  setpoint.q_n2sp.q2 = sqrt(2.0)/2.0;
  setpoint.q_n2sp.q3 = 0.0;
  
  // rotate by heading
  quat_t q_heading = {cos(0.5*setpoint.setpoint_heading), -sin(0.5*setpoint.setpoint_heading), 0.0,0.0};
  setpoint.q_n2sp = quat_mult_ret(setpoint.q_n2sp, q_heading);
 
  // rotate by stick commands
  double total_angle = sqrt(pitch_setpoint*pitch_setpoint + yaw_setpoint*yaw_setpoint+1e-9);
  setpoint.q_pitch_yaw_setpoint.q0 = cos(0.5*total_angle);
  setpoint.q_pitch_yaw_setpoint.q1 = 0.0;
  setpoint.q_pitch_yaw_setpoint.q2 = sin(0.5*total_angle)*pitch_setpoint/total_angle;
  setpoint.q_pitch_yaw_setpoint.q3 = sin(0.5*total_angle)*yaw_setpoint/total_angle;
  setpoint.q_n2sp = quat_mult_ret(setpoint.q_n2sp, setpoint.q_pitch_yaw_setpoint);

  // calculate body to setpoint quat
  quat_inv_mult( &(setpoint.q_b2sp), q_n2b, &(setpoint.q_n2sp));

  // output "pitch"/"yaw" estimated quat
  quat_mult_inv(&(setpoint.q_pitch_yaw_estimated), &(setpoint.q_pitch_yaw_setpoint), 
                &(setpoint.q_b2sp));

  // set paparazzi setpoint
  set_paparazzi_setpoint();
}



void
toytronics_set_sp_absolute_forward_from_rc()
{
  double dt = 1.0/RC_UPDATE_FREQ;
  const rc_t * const rc = get_rc();
  const quat_t * const q_n2b = get_q_n2b();
  const euler_t * const e_n2b = get_e_n2b();

  setpoint_old = setpoint;
  
  // local copies to allow implementing a deadband
  double rcp = rc->pitch;
  double rcr = apply_deadband(rc->roll, SETPOINT_DEADBAND);
  double rcy = apply_deadband(rc->yaw, SETPOINT_DEADBAND);

  euler_t e_n2sp;
  e_n2sp.pitch = rcp * SETPOINT_MAX_STICK_ANGLE_DEG * M_PI/180.0;
  e_n2sp.roll  = rcr * SETPOINT_MAX_STICK_ANGLE_DEG * M_PI/180.0;

  // integrate stick to get setpoint heading
  setpoint.setpoint_heading += dt*SETPOINT_MAX_STICK_DEG_PER_SEC*M_PI/180.0*rcy;
  setpoint.setpoint_heading += dt*e_n2sp.roll*roll_to_yaw_rate_ff_factor;

  wrap_to_pi( &setpoint.setpoint_heading );

  // bound setpoint heading
  setpoint.estimated_heading = get_fudged_yaw( q_n2b, e_n2b );
  double error_heading = setpoint.setpoint_heading - setpoint.estimated_heading;
  wrap_to_pi(&error_heading);
  BOUND(error_heading, -setpoint_absolute_heading_bound_deg*M_PI/180.0, setpoint_absolute_heading_bound_deg*M_PI/180.0);
  setpoint.setpoint_heading = setpoint.estimated_heading + error_heading;  

  // generate setpoint
  e_n2sp.yaw = setpoint.setpoint_heading;
  quat_of_euler321( &setpoint.q_n2sp, &e_n2sp );

  // smooth transition
  smooth_transition_angle -= dt*M_PI/180.0*90;
  if (smooth_transition_angle < 0) smooth_transition_angle = 0.0;

  quat_t q_st;
  q_st.q0 = cos(0.5*smooth_transition_angle);
  q_st.q1 = 0.0;
  q_st.q2 = sin(0.5*smooth_transition_angle);
  q_st.q3 = 0.0;
  setpoint.q_n2sp = quat_mult_ret(setpoint.q_n2sp, q_st);

  // calculate body to setpoint quat
  quat_inv_mult( &(setpoint.q_b2sp), q_n2b, &(setpoint.q_n2sp));

  // set paparazzi setpoint
  set_paparazzi_setpoint();
}




void
toytronics_set_sp_incremental_from_rc()
{
  double dt = 1.0/RC_UPDATE_FREQ;
  const rc_t * const rc = get_rc();
  const quat_t * const q_n2b = get_q_n2b();

  setpoint_old = setpoint;

  // local copies to allow implementing a deadband
  double rcp = apply_deadband(rc->pitch, SETPOINT_DEADBAND);
  double rcr = apply_deadband(rc->roll, SETPOINT_DEADBAND);
  double rcy = apply_deadband(rc->yaw, SETPOINT_DEADBAND);
  
  // rotation vector in body frame
  xyz_t w_dt_body = {rcr * SETPOINT_MAX_STICK_DEG_PER_SEC*M_PI/180.0*dt,
                     rcp * SETPOINT_MAX_STICK_DEG_PER_SEC*M_PI/180.0*dt,
                     rcy * SETPOINT_MAX_STICK_DEG_PER_SEC*M_PI/180.0*dt};
 
  // old body to setpoint quat q_b2sp
  quat_inv_mult( &(setpoint.q_b2sp), q_n2b, &(setpoint.q_n2sp));
  
  // rotation vector in setpoint frame
  xyz_t w_dt_sp;
  rot_vec_by_quat_a2b( &w_dt_sp, &(setpoint.q_b2sp), &w_dt_body);

  // form diff quat
  double total_angle = sqrt( w_dt_sp.x*w_dt_sp.x 
                             + w_dt_sp.y*w_dt_sp.y 
                             + w_dt_sp.z*w_dt_sp.z 
                             + 1e-9);
  quat_t diff_quat = {cos(total_angle/2.0),
                      sin(total_angle/2.0)*w_dt_sp.x/total_angle,
                      sin(total_angle/2.0)*w_dt_sp.y/total_angle,
                      sin(total_angle/2.0)*w_dt_sp.z/total_angle};

  // use diff quat to update setpoint quat
  setpoint.q_n2sp = quat_mult_ret(setpoint.q_n2sp, diff_quat);

  // calculate body to setpoint quat
  quat_inv_mult( &(setpoint.q_b2sp), q_n2b, &(setpoint.q_n2sp));
  
  // now bound setpoint quat to not get too far away from estimated quat
  BOUND(setpoint.q_b2sp.q1, -setpoint_incremental_bounds_deg.x*M_PI/180.0/2.0, setpoint_incremental_bounds_deg.x*M_PI/180.0/2.0);
  BOUND(setpoint.q_b2sp.q2, -setpoint_incremental_bounds_deg.y*M_PI/180.0/2.0, setpoint_incremental_bounds_deg.y*M_PI/180.0/2.0);
  BOUND(setpoint.q_b2sp.q3, -setpoint_incremental_bounds_deg.z*M_PI/180.0/2.0, setpoint_incremental_bounds_deg.z*M_PI/180.0/2.0);
  setpoint.q_b2sp.q0 = sqrt(1 - setpoint.q_b2sp.q1*setpoint.q_b2sp.q1 
 			    - setpoint.q_b2sp.q2*setpoint.q_b2sp.q2 
			    - setpoint.q_b2sp.q3*setpoint.q_b2sp.q3);
          
  // update n2sp quat
  quat_mult( &(setpoint.q_n2sp), q_n2b, &(setpoint.q_b2sp));

  //if bound is zero, then turn joystick into equiv. error
  // MUST BE DONE AFTER q_n2sp is updated
  if(setpoint_incremental_bounds_deg.x == 0.0) setpoint.q_b2sp.q1 = + rcr * SETPOINT_MAX_STICK_ANGLE_DEG*M_PI/180.0/2.0;
  if(setpoint_incremental_bounds_deg.y == 0.0) setpoint.q_b2sp.q2 = + rcp * SETPOINT_MAX_STICK_ANGLE_DEG*M_PI/180.0/2.0;
  if(setpoint_incremental_bounds_deg.z == 0.0) setpoint.q_b2sp.q3 = + rcy * SETPOINT_MAX_STICK_ANGLE_DEG*M_PI/180.0/2.0;

  // set paparazzi setpoint
  set_paparazzi_setpoint();
}


void
toytronics_mode_changed(int new_mode)
{
  switch ( new_mode ) {

  case GUIDANCE_H_MODE_TOYTRONICS_HOVER:
    #ifdef TOYTRONICS_HOVER_BYPASS_ROLL
    toytronics_sp_set_absolute_heading_bound_deg( 0.0 );
    #else
    toytronics_sp_set_absolute_heading_bound_deg( SETPOINT_BOUND_ERROR_HEADING_DEG );
    #endif
    // initialize setpoint heading to current heading
    toytronics_sp_enter_absolute_hover();
    // reset inner's integral state
    inner_setpoint_reset();
    inner_set_hover_gains();
    break;


  case GUIDANCE_H_MODE_TOYTRONICS_FORWARD:
    inner_set_forward_gains();
    toytronics_sp_set_incremental_bounds_deg( 0.0, 
                                              SETPOINT_MODE_2_BOUND_QUAT_DEG_Y, 
                                              SETPOINT_MODE_2_BOUND_QUAT_DEG_Z);
    toytronics_sp_enter_incremental();
    // initialize setpoint heading to current heading
    toytronics_sp_enter_absolute_forward();
    // init smooth transition
    setpoint_smooth_transition_reset();
    // reset inner's integral state
    inner_setpoint_reset();
    toytronics_sp_set_absolute_heading_bound_deg( SETPOINT_BOUND_ERROR_HEADING_DEG ); 
    break;


  case GUIDANCE_H_MODE_TOYTRONICS_ACRO:
    #ifdef TOYTRONICS_ACRO_BYPASS_ROLL
    toytronics_sp_set_incremental_bounds_deg( 0.0, 
                                              SETPOINT_MODE_2_BOUND_QUAT_DEG_Y, 
                                              SETPOINT_MODE_2_BOUND_QUAT_DEG_Z);
    #else
    toytronics_sp_set_incremental_bounds_deg( SETPOINT_MODE_2_BOUND_QUAT_DEG_X,
                                              SETPOINT_MODE_2_BOUND_QUAT_DEG_Y,
                                              SETPOINT_MODE_2_BOUND_QUAT_DEG_Z);
    #endif
    toytronics_sp_enter_incremental();
    inner_setpoint_reset();
    inner_set_aerobatic_gains();
    break;

  default:
    break;
  }
}
