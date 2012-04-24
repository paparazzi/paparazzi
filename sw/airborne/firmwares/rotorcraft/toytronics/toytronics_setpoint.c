// toytronics_setpoint.c
// Copyright (C) 2011, Greg Horn, Joby Robotics, LLC.
// Copyright (C) 2011, Pranay Sinha, Transition Robotics, Inc.
#include <math.h>
#include <string.h>
#include <stdio.h>

#include "toytronics_setpoint.h"
#include "toytronics_interface.h"
#include "toytronics_types.h"

#include "generated/airframe.h" // for aircraft constants

#define RC_UPDATE_FREQ 40

setpoint_t setpoint;

xyz_t setpoint_incremental_bounds_deg = { SETPOINT_MODE_2_BOUND_QUAT_DEG_X,
                                          SETPOINT_MODE_2_BOUND_QUAT_DEG_Y,
                                          SETPOINT_MODE_2_BOUND_QUAT_DEG_Z};

xyz_t setpoint_aerobatic_decay_time = {
   SETPOINT_AEROBATIC_DECAY_TIME_X, SETPOINT_AEROBATIC_DECAY_TIME_Y, SETPOINT_AEROBATIC_DECAY_TIME_Z};

xyz_t setpoint_rc_sensitivity = {
   SETPOINT_RC_SENSITIVITY_X, SETPOINT_RC_SENSITIVITY_Y, SETPOINT_RC_SENSITIVITY_Z};

double setpoint_absolute_heading_bound_deg = SETPOINT_BOUND_ERROR_HEADING_DEG;
double hover_pitch_trim_deg = SETPOINT_HOVER_PITCH_TRIM_DEG;

double tc_fading_upper_deg = SETPOINT_TC_FADING_UPPER_DEG;
double tc_fading_lower_deg = SETPOINT_TC_FADING_LOWER_DEG;

#ifdef AUTOPILOT_LOBATT_WING_WAGGLE
  double lobatt_wing_waggle_deg = SETPOINT_LOBATT_WING_WAGGLE_DEG; //angle to which wings are waggled
  double lobatt_wing_waggle_max = SETPOINT_LOBATT_WING_WAGGLE_MAX; //max number of waggles in each block
  double lobatt_wing_waggle_dt  = SETPOINT_LOBATT_WING_WAGGLE_DT;  //time multiplier for waggle spike
  bool_t setpoint_lobatt_wing_waggle_left = TRUE; //keep track of whether last wing waggle was to the left or right
  double setpoint_lobatt_wing_waggle_num = 1001; //keep track of number of waggles
#endif
double roll_body;

/**************** gains for the 3 modes ******************/
struct Int32AttitudeGains toytronics_hover_gains = {
  {TOYTRONICS_STAB_GAINS_HOVER_X_P,  TOYTRONICS_STAB_GAINS_HOVER_Y_P,  TOYTRONICS_STAB_GAINS_HOVER_Z_P  },
  {TOYTRONICS_STAB_GAINS_HOVER_X_D,  TOYTRONICS_STAB_GAINS_HOVER_Y_D,  TOYTRONICS_STAB_GAINS_HOVER_Z_D  },
  {TOYTRONICS_STAB_GAINS_HOVER_X_DD, TOYTRONICS_STAB_GAINS_HOVER_Y_DD, TOYTRONICS_STAB_GAINS_HOVER_Z_DD },
  {TOYTRONICS_STAB_GAINS_HOVER_X_I,  TOYTRONICS_STAB_GAINS_HOVER_Y_I,  TOYTRONICS_STAB_GAINS_HOVER_Z_I  }
};

struct Int32AttitudeGains toytronics_forward_gains = {
  {TOYTRONICS_STAB_GAINS_FORWARD_X_P,  TOYTRONICS_STAB_GAINS_FORWARD_Y_P,  TOYTRONICS_STAB_GAINS_FORWARD_Z_P  },
  {TOYTRONICS_STAB_GAINS_FORWARD_X_D,  TOYTRONICS_STAB_GAINS_FORWARD_Y_D,  TOYTRONICS_STAB_GAINS_FORWARD_Z_D  },
  {TOYTRONICS_STAB_GAINS_FORWARD_X_DD, TOYTRONICS_STAB_GAINS_FORWARD_Y_DD, TOYTRONICS_STAB_GAINS_FORWARD_Z_DD },
  {TOYTRONICS_STAB_GAINS_FORWARD_X_I,  TOYTRONICS_STAB_GAINS_FORWARD_Y_I,  TOYTRONICS_STAB_GAINS_FORWARD_Z_I  }
};

struct Int32AttitudeGains toytronics_aerobatic_gains = {
  {TOYTRONICS_STAB_GAINS_AEROBATIC_X_P,  TOYTRONICS_STAB_GAINS_AEROBATIC_Y_P,  TOYTRONICS_STAB_GAINS_AEROBATIC_Z_P  },
  {TOYTRONICS_STAB_GAINS_AEROBATIC_X_D,  TOYTRONICS_STAB_GAINS_AEROBATIC_Y_D,  TOYTRONICS_STAB_GAINS_AEROBATIC_Z_D  },
  {TOYTRONICS_STAB_GAINS_AEROBATIC_X_DD, TOYTRONICS_STAB_GAINS_AEROBATIC_Y_DD, TOYTRONICS_STAB_GAINS_AEROBATIC_Z_DD },
  {TOYTRONICS_STAB_GAINS_AEROBATIC_X_I,  TOYTRONICS_STAB_GAINS_AEROBATIC_Y_I,  TOYTRONICS_STAB_GAINS_AEROBATIC_Z_I  }
};


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
double aerobatic_accel_tc_gain = SETPOINT_AEROBATIC_ACCEL_TURN_COORDINATION_GAIN;
double hover_forward_accel_tc_gain = SETPOINT_HOVER_FORWARD_ACCEL_TURN_COORDINATION_GAIN;
double forward_accel_tc_gain = SETPOINT_FORWARD_ACCEL_TURN_COORDINATION_GAIN;
double smooth_transition_angle = 0.0;
double absolute_forward_pitch_trim_deg = SETPOINT_ABSOLUTE_FORWARD_PITCH_TRIM_DEG;

#define RC_INCREMENTAL_DEADBAND 0.02

/**************** prototypes *******************/
static void setpoint_smooth_transition_reset(void);

static void toytronics_sp_enter_incremental(void);
static void toytronics_sp_enter_absolute_hover(void);
static void toytronics_sp_enter_absolute_forward(void);
static void toytronics_sp_enter_hover_forward(void);

static void toytronics_sp_set_absolute_heading_bound_deg(double new_bound);
static void toytronics_sp_set_incremental_bounds_deg(double bound_x, double bound_y, double bound_z);

static double tc_fading(const quat_t * const q_n2b);

/****************** math/helper functions ******************/

// wrap to (-pi,pi)
static void
wrap_to_pi(double * angle){
  while (*angle > M_PI)
    *angle -= 2*M_PI;
  while (*angle < -M_PI)
    *angle += 2*M_PI;
}

// discrete time exponential decay
static void
discrete_exponential_decay( double * state, const double tau, const double ts )
{
  if(fabs(*state)>0.08)
    {*state *= exp(-ts/tau);}
}

// rc sticks power sensitivity function
static void
rc_sensitizer( double * state, const double sens )
{
  *state = copysign(pow(*state,sens),*state);
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

static double
hover_forward_yaw_of_quat(const quat_t * const q)
{

  double mr21 = -2*(q->q1*q->q2 - q->q0*q->q3);
  double r22 = q->q0*q->q0 - q->q1*q->q1 + q->q2*q->q2 - q->q3*q->q3;

  return atan2( mr21, r22 );
}

static double
hover_forward_roll_of_quat(const quat_t * const q)
{
  double r23 = 2*(q->q2*q->q3 + q->q0*q->q1);
  BOUND(r23, -1, 1);
  return asin(r23);
}

static double
tc_fading(const quat_t * const q_n2b)
{
  // linear interpolation - no dead band
  /*
  double r13 = 2.0*(q_n2b->q1*q_n2b->q3 - q_n2b->q0*q_n2b->q2);
  double tc_slider = 1 - abs(r13);
  */

  // deadband
  double r13 = 2.0*(q_n2b->q1*q_n2b->q3 - q_n2b->q0*q_n2b->q2);
  double abs_pitch_deg = abs(asin(-r13))*180.0/M_PI;
  double tc_slider;
  if (abs_pitch_deg > tc_fading_upper_deg)
    tc_slider = 0;
  else if (abs_pitch_deg < tc_fading_lower_deg)
    tc_slider = 1;
  else
    tc_slider = 1 - (abs_pitch_deg - tc_fading_lower_deg)/(tc_fading_upper_deg - tc_fading_lower_deg);

  return tc_slider;
}


/****************** enter mode functions ******************/
static void
setpoint_smooth_transition_reset()
{
  const quat_t * q_n2b = get_q_n2b();
  const euler_t * e_n2b = get_e_n2b();
  smooth_transition_angle = get_full_range_pitch(q_n2b, e_n2b);
  smooth_transition_angle -= absolute_forward_pitch_trim_deg*M_PI/180.0;
}

static void
toytronics_sp_enter_incremental()
{
  const quat_t * q_n2b = get_q_n2b();

  // copy current attitude to setpoint attitude
  memcpy( &(setpoint.q_n2sp), q_n2b, sizeof(quat_t));
}

static void
toytronics_sp_enter_absolute_hover()
{
  const quat_t * q_n2b = get_q_n2b();

  // copy current heading to setpoint heading
  setpoint.setpoint_heading = get_heading_from_q_n2b(q_n2b);
  memcpy( &(setpoint.q_n2sp), q_n2b, sizeof(quat_t));
}

static void
toytronics_sp_enter_hover_forward()
{
  const quat_t * q_n2b = get_q_n2b();

  // copy current heading to setpoint heading
  setpoint.setpoint_heading = hover_forward_yaw_of_quat( q_n2b );
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

  // set stabilization setpoint
  set_stabilization_setpoint(&setpoint.q_n2sp);
}

void
toytronics_set_sp_hover_forward_from_rc()
{
  double dt = 1.0/RC_UPDATE_FREQ;
  const rc_t * const rc = get_rc();
  const quat_t * const q_n2b = get_q_n2b();

  // estimated heading for telemetry and bounding
  setpoint.estimated_heading = hover_forward_yaw_of_quat( q_n2b );

  // local copies to allow implementing a deadband
  double rcp = rc->pitch;
  double rcr = rc->roll;
  double rcy = apply_deadband(rc->yaw, SETPOINT_DEADBAND);

  //****************rc sticks sensitivity adjustment****************
  rc_sensitizer(&rcr, setpoint_rc_sensitivity.x);
  rc_sensitizer(&rcp, setpoint_rc_sensitivity.y);
  rc_sensitizer(&rcy, setpoint_rc_sensitivity.z);
  //****************rc sticks sensitivity adjustment****************


  // set pitch/yaw from stick
  double pitch_body = (rcp * SETPOINT_MAX_STICK_ANGLE_DEG + hover_pitch_trim_deg + (fabs(rcr * SETPOINT_MAX_STICK_ANGLE_DEG)*(5/90) * fabs(rcp * SETPOINT_MAX_STICK_ANGLE_DEG)*(1/90)))*M_PI/180.0;

  #ifdef AUTOPILOT_LOBATT_WING_WAGGLE
    if (setpoint_lobatt_wing_waggle_num < lobatt_wing_waggle_max){
      if (setpoint_lobatt_wing_waggle_left==TRUE){
        if(roll_body < rcr * SETPOINT_MAX_STICK_ANGLE_DEG*M_PI/180.0 + lobatt_wing_waggle_deg*M_PI/180.0){roll_body += lobatt_wing_waggle_dt*dt*SETPOINT_MAX_STICK_DEG_PER_SEC*M_PI/180.0;}
        else{roll_body = rcr * SETPOINT_MAX_STICK_ANGLE_DEG*M_PI/180.0; setpoint_lobatt_wing_waggle_left=FALSE;setpoint_lobatt_wing_waggle_num+=1;}}
      else{
        if(roll_body > rcr * SETPOINT_MAX_STICK_ANGLE_DEG*M_PI/180.0 - lobatt_wing_waggle_deg*M_PI/180.0){roll_body -= lobatt_wing_waggle_dt*dt*SETPOINT_MAX_STICK_DEG_PER_SEC*M_PI/180.0;}
        else{roll_body = rcr * SETPOINT_MAX_STICK_ANGLE_DEG*M_PI/180.0; setpoint_lobatt_wing_waggle_left=TRUE;setpoint_lobatt_wing_waggle_num+=1;}}
    }
    else{
      roll_body   = rcr * SETPOINT_MAX_STICK_ANGLE_DEG*M_PI/180.0;}
  #else
    roll_body   = rcr * SETPOINT_MAX_STICK_ANGLE_DEG*M_PI/180.0;
  #endif

  // integrate stick to get setpoint heading
  setpoint.setpoint_heading += dt*SETPOINT_MAX_STICK_DEG_PER_SEC*M_PI/180.0*rcy;

  // body roll feedforward turn coordination
  #define START_FADING_DEG -50
  #define FINISH_FADING_DEG -70
  double ff_fading_slider;
  if (pitch_body > START_FADING_DEG*M_PI/180.0)
    ff_fading_slider = 0;
  else if (pitch_body < FINISH_FADING_DEG*M_PI/180.0)
    ff_fading_slider = 1;
  else
    ff_fading_slider = (pitch_body*180.0/M_PI - START_FADING_DEG)/(FINISH_FADING_DEG - START_FADING_DEG);

  setpoint.setpoint_heading += dt*roll_body*roll_to_yaw_rate_ff_factor*ff_fading_slider;

  // accel y turn coordination
  setpoint.setpoint_heading -= dt*tc_fading(q_n2b)*hover_forward_accel_tc_gain*get_y_accel();

  // bound heading error
  double heading_error = setpoint.setpoint_heading - setpoint.estimated_heading;
  wrap_to_pi(&heading_error);
  BOUND(heading_error, -1.0, 1.0);
  //********heading error decay*****************
  discrete_exponential_decay( &heading_error, setpoint_aerobatic_decay_time.x, dt );
  //********************************************
  setpoint.setpoint_heading = setpoint.estimated_heading + heading_error;
  wrap_to_pi(&setpoint.setpoint_heading);

  // start straight up
  quat_t q_y90 = {sqrt(2)/2.0, 0, sqrt(2)/2.0, 0};
  quat_memcpy( &setpoint.q_n2sp, &q_y90 );

  // yaw
  quat_t q_yaw = {1,0,0,0};
  q_yaw.q0 =  cos(0.5*setpoint.setpoint_heading);
  q_yaw.q1 = -sin(0.5*setpoint.setpoint_heading);
  quat_t q_temp;
  quat_memcpy( &q_temp, &setpoint.q_n2sp );
  quat_mult( &setpoint.q_n2sp, &q_temp, &q_yaw );

  // roll
  quat_t q_roll = {1,0,0,0};
  q_roll.q0 = cos(0.5*roll_body);
  q_roll.q3 = sin(0.5*roll_body);
  quat_memcpy( &q_temp, &setpoint.q_n2sp );
  quat_mult( &setpoint.q_n2sp, &q_temp, &q_roll );

  // pitch
  quat_t q_pitch = {1,0,0,0};
  q_pitch.q0 = cos(0.5*pitch_body);
  q_pitch.q2 = sin(0.5*pitch_body);
  quat_memcpy( &q_temp, &setpoint.q_n2sp );
  quat_mult( &setpoint.q_n2sp, &q_temp, &q_pitch );

  /* // calculate body to setpoint quat */
  /* quat_inv_mult( &(setpoint.q_b2sp), q_n2b, &(setpoint.q_n2sp)); */

  /* // now bound setpoint quat to not get too far away from estimated quat */
  /* BOUND(setpoint.q_b2sp.q1, -setpoint_incremental_bounds_deg.x*M_PI/180.0/2.0, setpoint_incremental_bounds_deg.x*M_PI/180.0/2.0); */
  /* BOUND(setpoint.q_b2sp.q2, -setpoint_incremental_bounds_deg.y*M_PI/180.0/2.0, setpoint_incremental_bounds_deg.y*M_PI/180.0/2.0); */
  /* BOUND(setpoint.q_b2sp.q3, -setpoint_incremental_bounds_deg.z*M_PI/180.0/2.0, setpoint_incremental_bounds_deg.z*M_PI/180.0/2.0); */

  /* // let setpoint decay back to body */
  /* discrete_exponential_decay( &setpoint.q_b2sp.q1, setpoint_aerobatic_decay_time.x, dt ); */
  /* discrete_exponential_decay( &setpoint.q_b2sp.q2, setpoint_aerobatic_decay_time.y, dt ); */
  /* discrete_exponential_decay( &setpoint.q_b2sp.q3, setpoint_aerobatic_decay_time.z, dt ); */

  /* // normalize */
  /* setpoint.q_b2sp.q0 = sqrt(1 - SQR(setpoint.q_b2sp.q1) - SQR(setpoint.q_b2sp.q2) - SQR(setpoint.q_b2sp.q3)); */

  /* // update n2sp quat */
  /* quat_mult( &(setpoint.q_n2sp), q_n2b, &(setpoint.q_b2sp)); */

  // update setpoint.heading
  setpoint.setpoint_heading = hover_forward_yaw_of_quat( &setpoint.q_n2sp );

  // set stabilization setpoint
  set_stabilization_setpoint(&setpoint.q_n2sp);
}



void
toytronics_set_sp_absolute_forward_from_rc()
{
  double dt = 1.0/RC_UPDATE_FREQ;
  const rc_t * const rc = get_rc();
  const quat_t * const q_n2b = get_q_n2b();
  const euler_t * const e_n2b = get_e_n2b();

  // local copies to allow implementing a deadband
  double rcp = rc->pitch;
  double rcr = apply_deadband(rc->roll, SETPOINT_DEADBAND);
  double rcy = apply_deadband(rc->yaw, SETPOINT_DEADBAND);

  //****************rc sticks sensitivity adjustment****************
  rc_sensitizer(&rcr, setpoint_rc_sensitivity.x);
  rc_sensitizer(&rcp, setpoint_rc_sensitivity.y);
  rc_sensitizer(&rcy, setpoint_rc_sensitivity.z);
  //****************rc sticks sensitivity adjustment****************

  #ifdef AUTOPILOT_LOBATT_WING_WAGGLE
    if (setpoint_lobatt_wing_waggle_num < lobatt_wing_waggle_max){
      if (setpoint_lobatt_wing_waggle_left==TRUE){
        if(roll_body < rcr * SETPOINT_MAX_STICK_ANGLE_DEG*M_PI/180.0 + 2*lobatt_wing_waggle_deg*M_PI/180.0){roll_body += lobatt_wing_waggle_dt*dt*SETPOINT_MAX_STICK_DEG_PER_SEC*M_PI/180.0;}
        else{roll_body = rcr * SETPOINT_MAX_STICK_ANGLE_DEG*M_PI/180.0; setpoint_lobatt_wing_waggle_left=FALSE;setpoint_lobatt_wing_waggle_num+=1;}}
      else{
        if(roll_body > rcr * SETPOINT_MAX_STICK_ANGLE_DEG*M_PI/180.0 - 2*lobatt_wing_waggle_deg*M_PI/180.0){roll_body -= lobatt_wing_waggle_dt*dt*SETPOINT_MAX_STICK_DEG_PER_SEC*M_PI/180.0;}
        else{roll_body = rcr * SETPOINT_MAX_STICK_ANGLE_DEG*M_PI/180.0; setpoint_lobatt_wing_waggle_left=TRUE;setpoint_lobatt_wing_waggle_num+=1;}}
    }
    else{
      roll_body   = rcr * SETPOINT_MAX_STICK_ANGLE_DEG*M_PI/180.0;}
  #else
    roll_body   = rcr * SETPOINT_MAX_STICK_ANGLE_DEG*M_PI/180.0;
  #endif

  euler_t e_n2sp;
  e_n2sp.pitch = (rcp * SETPOINT_MAX_STICK_ANGLE_DEG + absolute_forward_pitch_trim_deg+ fabs(rcr * SETPOINT_MAX_STICK_ANGLE_DEG)*(5/90))*M_PI/180.0;
  e_n2sp.roll  = roll_body; //rcr * SETPOINT_MAX_STICK_ANGLE_DEG * M_PI/180.0;

  // integrate stick to get setpoint heading
  setpoint.setpoint_heading += dt*SETPOINT_MAX_STICK_DEG_PER_SEC*M_PI/180.0*rcy;
  setpoint.setpoint_heading += dt*e_n2sp.roll*roll_to_yaw_rate_ff_factor;

  // accel turn coordination
  setpoint.setpoint_heading -= dt*tc_fading(q_n2b)*forward_accel_tc_gain*get_y_accel();

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
  smooth_transition_angle -= dt*M_PI/180.0*SETPOINT_MODE_1_2_TRANSITION_DEGREES_PER_SEC;
  if (smooth_transition_angle < 0) smooth_transition_angle = 0.0;

  quat_t q_st;
  q_st.q0 = cos(0.5*smooth_transition_angle);
  q_st.q1 = 0.0;
  q_st.q2 = sin(0.5*smooth_transition_angle);
  q_st.q3 = 0.0;
  setpoint.q_n2sp = quat_mult_ret(setpoint.q_n2sp, q_st);

  // calculate body to setpoint quat
  quat_inv_mult( &(setpoint.q_b2sp), q_n2b, &(setpoint.q_n2sp));

  // set stabilization setpoint
  set_stabilization_setpoint(&setpoint.q_n2sp);
}

void
toytronics_set_sp_incremental_from_rc()
{
  double dt = 1.0/RC_UPDATE_FREQ;
  const rc_t * const rc = get_rc();
  const quat_t * const q_n2b = get_q_n2b();

  // local copies to allow implementing a deadband
  double rcp = apply_deadband(rc->pitch, SETPOINT_DEADBAND);
  double rcr = apply_deadband(rc->roll, SETPOINT_DEADBAND);
  double rcy = apply_deadband(rc->yaw, SETPOINT_DEADBAND);

  //****************rc sticks sensitivity adjustment****************
  rc_sensitizer(&rcr, setpoint_rc_sensitivity.x);
  rc_sensitizer(&rcp, setpoint_rc_sensitivity.y);
  rc_sensitizer(&rcy, setpoint_rc_sensitivity.z);
  //****************rc sticks sensitivity adjustment****************

  #ifdef AUTOPILOT_LOBATT_WING_WAGGLE
    if (setpoint_lobatt_wing_waggle_num < lobatt_wing_waggle_max){
      if (setpoint_lobatt_wing_waggle_left==TRUE){
        if(roll_body < rcr * SETPOINT_MAX_STICK_ANGLE_DEG*M_PI/180.0 + 2*lobatt_wing_waggle_deg*M_PI/180.0){roll_body += lobatt_wing_waggle_dt*dt*SETPOINT_MAX_STICK_DEG_PER_SEC*M_PI/180.0;}
        else{roll_body = rcr * SETPOINT_MAX_STICK_ANGLE_DEG*M_PI/180.0; setpoint_lobatt_wing_waggle_left=FALSE;setpoint_lobatt_wing_waggle_num+=1;}}
      else{
        if(roll_body > rcr * SETPOINT_MAX_STICK_ANGLE_DEG*M_PI/180.0 - 2*lobatt_wing_waggle_deg*M_PI/180.0){roll_body -= lobatt_wing_waggle_dt*dt*SETPOINT_MAX_STICK_DEG_PER_SEC*M_PI/180.0;}
        else{roll_body = rcr * SETPOINT_MAX_STICK_ANGLE_DEG*M_PI/180.0; setpoint_lobatt_wing_waggle_left=TRUE;setpoint_lobatt_wing_waggle_num+=1;}}
    }
    else{
      roll_body   = rcr * SETPOINT_MAX_STICK_ANGLE_DEG*M_PI/180.0;}
  #else
    roll_body = rcr * SETPOINT_MAX_STICK_DEG_PER_SEC*M_PI/180.0*dt;
  #endif

  // rotation vector in body frame
  xyz_t w_dt_body = {roll_body,//rcr * SETPOINT_MAX_STICK_DEG_PER_SEC*M_PI/180.0*dt,
                     rcp * SETPOINT_MAX_STICK_DEG_PER_SEC*M_PI/180.0*dt,
                     rcy * SETPOINT_MAX_STICK_DEG_PER_SEC*M_PI/180.0*dt};

  // try accelerometer turn coordination
  w_dt_body.z -= dt*tc_fading(q_n2b)*aerobatic_accel_tc_gain*get_y_accel();

  // old body to setpoint quat q_b2sp
  quat_inv_mult( &(setpoint.q_b2sp), q_n2b, &(setpoint.q_n2sp));

  // rotation vector in setpoint frame
  xyz_t w_dt_sp;
  rot_vec_by_quat_a2b( &w_dt_sp, &(setpoint.q_b2sp), &w_dt_body);

  // form diff quat
  double total_angle = sqrt( w_dt_sp.x*w_dt_sp.x 
                           + w_dt_sp.y*w_dt_sp.y 
                           + w_dt_sp.z*w_dt_sp.z);
  quat_t diff_quat;
  if (total_angle < 1e-12){
    diff_quat.q0 = 1;
    diff_quat.q1 = 0;
    diff_quat.q2 = 0;
    diff_quat.q3 = 0;
  } else {
    diff_quat.q0 = cos(total_angle/2.0);
    diff_quat.q1 = sin(total_angle/2.0)*w_dt_sp.x/total_angle;
    diff_quat.q2 = sin(total_angle/2.0)*w_dt_sp.y/total_angle;
    diff_quat.q3 = sin(total_angle/2.0)*w_dt_sp.z/total_angle;
  }

  // use diff quat to update setpoint quat
  setpoint.q_n2sp = quat_mult_ret(setpoint.q_n2sp, diff_quat);

  // calculate body to setpoint quat
  quat_inv_mult( &(setpoint.q_b2sp), q_n2b, &(setpoint.q_n2sp));

  // now bound setpoint quat to not get too far away from estimated quat
  BOUND(setpoint.q_b2sp.q1, -setpoint_incremental_bounds_deg.x*M_PI/180.0/2.0, setpoint_incremental_bounds_deg.x*M_PI/180.0/2.0);
  BOUND(setpoint.q_b2sp.q2, -setpoint_incremental_bounds_deg.y*M_PI/180.0/2.0, setpoint_incremental_bounds_deg.y*M_PI/180.0/2.0);
  BOUND(setpoint.q_b2sp.q3, -setpoint_incremental_bounds_deg.z*M_PI/180.0/2.0, setpoint_incremental_bounds_deg.z*M_PI/180.0/2.0);

  // let setpoint decay back to body
  discrete_exponential_decay( &setpoint.q_b2sp.q1, setpoint_aerobatic_decay_time.x, dt );
  discrete_exponential_decay( &setpoint.q_b2sp.q2, setpoint_aerobatic_decay_time.y, dt );
  discrete_exponential_decay( &setpoint.q_b2sp.q3, setpoint_aerobatic_decay_time.z, dt );

  // normalize
  setpoint.q_b2sp.q0 = sqrt(1 - SQR(setpoint.q_b2sp.q1) - SQR(setpoint.q_b2sp.q2) - SQR(setpoint.q_b2sp.q3));

  // update n2sp quat
  quat_mult( &(setpoint.q_n2sp), q_n2b, &(setpoint.q_b2sp));

  // set stabilization setpoint
  set_stabilization_setpoint(&setpoint.q_n2sp);
}

void
toytronics_mode_exit(int old_mode __attribute__((unused)))
{
  set_stabilization_gains( NULL );
}

void
toytronics_mode_enter(int new_mode)
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
    // set hover gains
    set_stabilization_gains(&toytronics_hover_gains);
    break;

  case GUIDANCE_H_MODE_TOYTRONICS_HOVER_FORWARD:
    toytronics_sp_set_absolute_heading_bound_deg( SETPOINT_BOUND_ERROR_HEADING_DEG );

    // initialize setpoint heading to current heading
    toytronics_sp_enter_hover_forward();

    // set hover gains
    set_stabilization_gains(&toytronics_hover_gains);
    break;

  case GUIDANCE_H_MODE_TOYTRONICS_FORWARD:
    toytronics_sp_set_incremental_bounds_deg( 0.0,
                                              SETPOINT_MODE_2_BOUND_QUAT_DEG_Y,
                                              SETPOINT_MODE_2_BOUND_QUAT_DEG_Z);
    toytronics_sp_set_absolute_heading_bound_deg( SETPOINT_BOUND_ERROR_HEADING_DEG );
    // initialize setpoint heading to current heading
    toytronics_sp_enter_absolute_forward();
    // init smooth transition
    setpoint_smooth_transition_reset();
    // set forward gains
    set_stabilization_gains(&toytronics_forward_gains);
    break;

  case GUIDANCE_H_MODE_TOYTRONICS_AEROBATIC:
    #ifdef TOYTRONICS_AEROBATIC_BYPASS_ROLL
    toytronics_sp_set_incremental_bounds_deg( 0.0,
                                              SETPOINT_MODE_2_BOUND_QUAT_DEG_Y,
                                              SETPOINT_MODE_2_BOUND_QUAT_DEG_Z);
    #else
    toytronics_sp_set_incremental_bounds_deg( SETPOINT_MODE_2_BOUND_QUAT_DEG_X,
                                              SETPOINT_MODE_2_BOUND_QUAT_DEG_Y,
                                              SETPOINT_MODE_2_BOUND_QUAT_DEG_Z);
    #endif
    toytronics_sp_enter_incremental();
    // set aerobatic gains
    set_stabilization_gains(&toytronics_aerobatic_gains);
    break;

  default:
    break;
  }
}
