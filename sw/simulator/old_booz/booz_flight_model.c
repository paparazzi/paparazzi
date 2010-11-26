#include "booz_flight_model.h"

#define BFMS_X     0
#define BFMS_Y     1
#define BFMS_Z     2
#define BFMS_XD    3
#define BFMS_YD    4
#define BFMS_ZD    5
#define BFMS_PHI   6
#define BFMS_THETA 7
#define BFMS_PSI   8
#define BFMS_P     9
#define BFMS_Q    10
#define BFMS_R    11
#define BFMS_OM_B 12
#define BFMS_OM_F 13
#define BFMS_OM_R 14
#define BFMS_OM_L 15
#define BFMS_SIZE 16

#define BoozFlighModelGetPos(_dest) {		\
    _dest->ve[AXIS_X] = bfm.state->ve[BFMS_X];	\
    _dest->ve[AXIS_Y] = bfm.state->ve[BFMS_Y];	\
    _dest->ve[AXIS_Z] = bfm.state->ve[BFMS_Z];	\
  }

#define BoozFlighModelGetSpeedLtp(_dest) {	\
    _dest->ve[AXIS_X] = bfm.state->ve[BFMS_XD];	\
    _dest->ve[AXIS_Y] = bfm.state->ve[BFMS_YD];	\
    _dest->ve[AXIS_Z] = bfm.state->ve[BFMS_ZD];	\
  }

#define BoozFlighModelGetAngles(_dest) {		\
    _dest->ve[EULER_PHI]   = bfm.state->ve[BFMS_PHI];	\
    _dest->ve[EULER_THETA] = bfm.state->ve[BFMS_THETA];	\
    _dest->ve[EULER_PSI]   = bfm.state->ve[BFMS_PSI];	\
  }

#define BoozFlighModelGetRate(_dest) {		\
    _dest->ve[AXIS_P] = bfm.state->ve[BFMS_P];	\
    _dest->ve[AXIS_Q] = bfm.state->ve[BFMS_Q];	\
    _dest->ve[AXIS_R] = bfm.state->ve[BFMS_R];	\
  }

#define BoozFlighModelGetRPMS(_dest) {			\
    _dest->ve[SERVO_BACK]  = bfm.state->ve[BFMS_OM_B];	\
    _dest->ve[SERVO_FRONT] = bfm.state->ve[BFMS_OM_F];	\
    _dest->ve[SERVO_RIGHT] = bfm.state->ve[BFMS_OM_R];	\
    _dest->ve[SERVO_LEFT]  = bfm.state->ve[BFMS_OM_L];	\
  }


#include <math.h>

#include "booz_flight_model_params.h"
#include "booz_flight_model_utils.h"
#include "booz_wind_model.h"

#include "6dof.h"


struct BoozFlightModel bfm;

//static void motor_model_derivative(VEC* x, VEC* u, VEC* xdot);

static void booz_flight_model_update_byproducts(void);
static VEC* booz_get_forces_ltp(VEC* F , VEC* speed_ltp, MAT* dcm_t, VEC* omega_square);
static VEC* booz_get_moments_body_frame(VEC* M, VEC* omega_square );
static void booz_flight_model_get_derivatives(VEC* X, VEC* u, VEC* Xdot);

void booz_flight_model_init( void ) {
  bfm.on_ground = TRUE;

  bfm.time = 0.;
  bfm.bat_voltage = BAT_VOLTAGE;
  bfm.mot_voltage = v_get(SERVOS_NB);

  bfm.state =  v_get(BFMS_SIZE);
  v_zero(bfm.state);

  bfm.pos_ltp   = v_get(AXIS_NB);
  bfm.speed_ltp = v_get(AXIS_NB);
  bfm.accel_ltp = v_get(AXIS_NB);

  bfm.speed_body = v_get(AXIS_NB);
  bfm.accel_body = v_get(AXIS_NB);

  bfm.eulers = v_get(AXIS_NB);
  bfm.ang_rate_body = v_get(AXIS_NB);
  bfm.ang_accel_body = v_get(AXIS_NB);

  bfm.dcm =  m_get(AXIS_NB, AXIS_NB);
  bfm.dcm_t =  m_get(AXIS_NB, AXIS_NB);
  bfm.quat = v_get(AXIS_NB);

  bfm.omega = v_get(SERVOS_NB);
  bfm.omega_square = v_get(SERVOS_NB);


  /* constants */
  bfm.g_ltp = v_get(AXIS_NB);
  bfm.g_ltp->ve[AXIS_X] = 0.;
  bfm.g_ltp->ve[AXIS_Y] = 0.;
  bfm.g_ltp->ve[AXIS_Z] = G;

  /* FIXME */
  bfm.h_ltp = v_get(AXIS_NB);
  bfm.h_ltp->ve[AXIS_X] = 1.;
  bfm.h_ltp->ve[AXIS_Y] = 0.;
  bfm.h_ltp->ve[AXIS_Z] = 1.;

  bfm.thrust_factor = 0.5 * RHO * PROP_AREA * C_t * PROP_RADIUS * PROP_RADIUS;
  bfm.torque_factor = 0.5 * RHO * PROP_AREA * C_q * PROP_RADIUS * PROP_RADIUS;

  bfm.props_moment_matrix = m_get(AXIS_NB, SERVOS_NB);
  m_zero(bfm.props_moment_matrix);
  bfm.props_moment_matrix->me[AXIS_X][SERVO_LEFT]  =  L * bfm.thrust_factor;
  bfm.props_moment_matrix->me[AXIS_X][SERVO_RIGHT] = -L * bfm.thrust_factor;
  bfm.props_moment_matrix->me[AXIS_Y][SERVO_BACK]  = -L * bfm.thrust_factor;
  bfm.props_moment_matrix->me[AXIS_Y][SERVO_FRONT] =  L * bfm.thrust_factor;
  bfm.props_moment_matrix->me[AXIS_Z][SERVO_LEFT]  =  bfm.torque_factor;
  bfm.props_moment_matrix->me[AXIS_Z][SERVO_RIGHT] =  bfm.torque_factor;
  bfm.props_moment_matrix->me[AXIS_Z][SERVO_BACK]  =  -bfm.torque_factor;
  bfm.props_moment_matrix->me[AXIS_Z][SERVO_FRONT] =  -bfm.torque_factor;

  bfm.mass = MASS;

  bfm.Inert = m_get(AXIS_NB, AXIS_NB);
  m_zero(bfm.Inert);
  bfm.Inert->me[AXIS_X][AXIS_X] = Ix;
  bfm.Inert->me[AXIS_Y][AXIS_Y] = Iy;
  bfm.Inert->me[AXIS_Z][AXIS_Z] = Iz;

  bfm.Inert_inv = m_get(AXIS_NB, AXIS_NB);
  m_zero(bfm.Inert_inv);
  bfm.Inert_inv->me[AXIS_X][AXIS_X] = 1./Ix;
  bfm.Inert_inv->me[AXIS_Y][AXIS_Y] = 1./Iy;
  bfm.Inert_inv->me[AXIS_Z][AXIS_Z] = 1./Iz;

}


#define WRAP(x,a) { while (x > a) x -= 2 * a; while (x <= -a) x += 2 * a;}

void booz_flight_model_run( double dt, double* commands ) {


  int i;
  for (i=0; i<SERVOS_NB; i++)
    bfm.mot_voltage->ve[i] = bfm.bat_voltage * commands[i];
  //  rk4(motor_model_derivative, bfm.mot_omega, bfm.mot_voltage, dt);
  rk4(booz_flight_model_get_derivatives, bfm.state, bfm.mot_voltage, dt);
  /* wrap euler angles */
  WRAP( bfm.state->ve[BFMS_PHI], M_PI);
  WRAP( bfm.state->ve[BFMS_THETA], M_PI_2);
  WRAP( bfm.state->ve[BFMS_PSI], M_PI);
  booz_flight_model_update_byproducts();
  bfm.time += dt;
}


static void booz_flight_model_update_byproducts(void) {

  /* extract eulers angles from state */
  BoozFlighModelGetAngles( bfm.eulers);
  /* extract body rotational rates from state */
  BoozFlighModelGetRate( bfm.ang_rate_body);

  /* direct cosine matrix ( inertial to body )*/
  dcm_of_eulers(bfm.eulers, bfm.dcm);
  /* transpose of dcm ( body to inertial ) */
  m_transp(bfm.dcm, bfm.dcm_t);
  /* quaternion */
  quat_of_eulers(bfm.quat, bfm.eulers);

  BoozFlighModelGetPos(bfm.pos_ltp);
  /* extract speed in ltp frame from state */
  BoozFlighModelGetSpeedLtp(bfm.speed_ltp);
  /* extract prop rotational speeds from state */
  BoozFlighModelGetRPMS(bfm.omega);
  /* compute square */
  v_star(bfm.omega, bfm.omega, bfm.omega_square);
  /* compute ltp accelerations */
  static VEC *f_ltp = VNULL;
  f_ltp = v_resize(f_ltp, AXIS_NB);
  f_ltp = booz_get_forces_ltp(f_ltp , bfm.speed_ltp, bfm.dcm_t, bfm.omega_square);
  sv_mlt( 1./bfm.mass, f_ltp, bfm.accel_ltp);
  /* rotate speed and accel to body frame */
  mv_mlt(bfm.dcm, bfm.speed_ltp, bfm.speed_body);
  mv_mlt(bfm.dcm, bfm.accel_ltp, bfm.accel_body);


  /* rotational accelerations  */


}



/*
   compute the sum of external forces.
   assumes that dcm and omega_square are already precomputed from X
*/
static VEC* booz_get_forces_ltp(VEC* F , VEC* speed_ltp, MAT* dcm_t, VEC* omega_square) {

  // FIXME : nimporte koi !
  F = v_zero(F);
  if (!bfm.on_ground) {

    // propeller thrust
    static VEC *prop_thrust_body = VNULL;
    prop_thrust_body = v_resize(prop_thrust_body, AXIS_NB);
    prop_thrust_body->ve[AXIS_X] = 0;
    prop_thrust_body->ve[AXIS_Y] = 0;
    prop_thrust_body->ve[AXIS_Z] = -v_sum(omega_square) * bfm.thrust_factor;
    static VEC *prop_thrust_ltp = VNULL;
    prop_thrust_ltp = v_resize(prop_thrust_ltp, AXIS_NB);
    prop_thrust_ltp = mv_mlt(dcm_t, prop_thrust_body, prop_thrust_ltp);
    F = v_add(F, prop_thrust_ltp, F);

    // gravity
    F = v_mltadd(F, bfm.g_ltp, bfm.mass, F);

    // drag
    static VEC *airspeed_ltp = VNULL;
    airspeed_ltp = v_resize(airspeed_ltp, AXIS_NB);
    airspeed_ltp = v_sub(speed_ltp, bwm.velocity, airspeed_ltp);
    double norm_speed = v_norm2(airspeed_ltp);
    F = v_mltadd(F, airspeed_ltp, -norm_speed * C_d_body, F);

  }
  return F;
}

/*
   compute the sum of external moments.
   assumes that omega_square is already precomputed from X
*/
static VEC* booz_get_moments_body_frame(VEC* M, VEC* omega_square ) {
  if (bfm.on_ground) {
    M = v_zero(M);
  }
  else {
    M =  mv_mlt(bfm.props_moment_matrix, omega_square, M);
  }
  return M;
}

static void booz_flight_model_get_derivatives(VEC* X, VEC* u, VEC* Xdot) {

  /* square of prop rotational speeds */
  static VEC *omega_square = VNULL;
  omega_square = v_resize(omega_square,SERVOS_NB);
  BoozFlighModelGetRPMS(omega_square);
  omega_square = v_star(omega_square, omega_square, omega_square);
  /* extract eulers angles from state */
  static VEC *eulers = VNULL;
  eulers = v_resize(eulers, AXIS_NB);
  BoozFlighModelGetAngles(eulers);
  /* direct cosine matrix ( inertial to body )*/
  static MAT *dcm = MNULL;
  dcm = m_resize(dcm,AXIS_NB, AXIS_NB);
  dcm = dcm_of_eulers(eulers, dcm);
  /* transpose of dcm ( body to inertial ) */
  static MAT *dcm_t = MNULL;
  dcm_t = m_resize(dcm_t,AXIS_NB, AXIS_NB);
  dcm_t = m_transp(dcm, dcm_t);
  /* extract ltp_speeds_from state */
  static VEC *speed_ltp = VNULL;
  speed_ltp = v_resize(speed_ltp, AXIS_NB);
  BoozFlighModelGetSpeedLtp(speed_ltp);
  /* extracts body rates from state */
  static VEC *rate_body = VNULL;
  rate_body = v_resize(rate_body, AXIS_NB);
  BoozFlighModelGetRate(rate_body);

  /* derivatives of position */
  Xdot->ve[BFMS_X] = speed_ltp->ve[AXIS_X];
  Xdot->ve[BFMS_Y] = speed_ltp->ve[AXIS_Y];
  Xdot->ve[BFMS_Z] = speed_ltp->ve[AXIS_Z];

  /* derivatives of speed           */
  static VEC *f_ltp = VNULL;
  f_ltp = v_resize(f_ltp, AXIS_NB);
  f_ltp = booz_get_forces_ltp(f_ltp , speed_ltp, dcm_t, omega_square);
  Xdot->ve[BFMS_XD] = 1./bfm.mass * f_ltp->ve[AXIS_X];
  Xdot->ve[BFMS_YD] = 1./bfm.mass * f_ltp->ve[AXIS_Y];
  Xdot->ve[BFMS_ZD] = 1./bfm.mass * f_ltp->ve[AXIS_Z];

  /* derivatives of eulers   */
  double sinPHI   = sin(eulers->ve[EULER_PHI]);
  double cosPHI   = cos(eulers->ve[EULER_PHI]);
  double cosTHETA = cos(eulers->ve[EULER_THETA]);
  double tanTHETA = tan(eulers->ve[EULER_THETA]);
  static MAT *euler_dot_of_pqr = MNULL;
  euler_dot_of_pqr = m_resize(euler_dot_of_pqr,AXIS_NB, AXIS_NB);
  euler_dot_of_pqr->me[EULER_PHI][AXIS_P] = 1.;
  euler_dot_of_pqr->me[EULER_PHI][AXIS_Q] = sinPHI*tanTHETA;
  euler_dot_of_pqr->me[EULER_PHI][AXIS_R] = cosPHI*tanTHETA;
  euler_dot_of_pqr->me[EULER_THETA][AXIS_P] = 0.;
  euler_dot_of_pqr->me[EULER_THETA][AXIS_Q] = cosPHI;
  euler_dot_of_pqr->me[EULER_THETA][AXIS_R] = -sinPHI;
  euler_dot_of_pqr->me[EULER_PSI][AXIS_P] = 0.;
  euler_dot_of_pqr->me[EULER_PSI][AXIS_Q] = sinPHI/cosTHETA;
  euler_dot_of_pqr->me[EULER_PSI][AXIS_R] = cosPHI/cosTHETA;
  static VEC *euler_dot = VNULL;
  euler_dot = v_resize(euler_dot, AXIS_NB);
  euler_dot = mv_mlt(euler_dot_of_pqr, rate_body, euler_dot);
  Xdot->ve[BFMS_PHI] = euler_dot->ve[EULER_PHI];
  Xdot->ve[BFMS_THETA] = euler_dot->ve[EULER_THETA];
  Xdot->ve[BFMS_PSI] = euler_dot->ve[EULER_PSI];

  /* derivatives of rates    */
  /* compute external moments */
  static VEC *m_body = VNULL;
  m_body = v_resize(m_body, AXIS_NB);
  m_body = booz_get_moments_body_frame(m_body, omega_square);
  /* Newton in body frame    */
  static VEC *i_omega = VNULL;
  i_omega = v_resize(i_omega, AXIS_NB);
  i_omega = mv_mlt(bfm.Inert, rate_body, i_omega);
  static VEC *omega_i_omega = VNULL;
  omega_i_omega = v_resize(omega_i_omega, AXIS_NB);
  omega_i_omega = out_prod(rate_body, i_omega, omega_i_omega);
  static VEC *m_tot = VNULL;
  m_tot = v_resize(m_tot, AXIS_NB);
  m_tot = v_sub(m_body, omega_i_omega, m_tot);
  static VEC *I_inv_m_tot = VNULL;
  I_inv_m_tot = v_resize(I_inv_m_tot, AXIS_NB);
  I_inv_m_tot = mv_mlt(bfm.Inert_inv, m_tot, I_inv_m_tot);
  Xdot->ve[BFMS_P] = I_inv_m_tot->ve[AXIS_P];
  Xdot->ve[BFMS_Q] = I_inv_m_tot->ve[AXIS_Q];
  Xdot->ve[BFMS_R] = I_inv_m_tot->ve[AXIS_R];

  /* derivatives of motors rpm */
  /* omega_dot = -1/THAU*omega - Kq*omega^2 + Kv/THAU * V */
  Xdot->ve[BFMS_OM_B] = -1./THAU * X->ve[BFMS_OM_B] - Kq * omega_square->ve[SERVO_BACK] + Kv/THAU * u->ve[SERVO_BACK];
  Xdot->ve[BFMS_OM_F] = -1./THAU * X->ve[BFMS_OM_F] - Kq * omega_square->ve[SERVO_FRONT] + Kv/THAU * u->ve[SERVO_FRONT];
  Xdot->ve[BFMS_OM_R] = -1./THAU * X->ve[BFMS_OM_R] - Kq * omega_square->ve[SERVO_RIGHT] + Kv/THAU * u->ve[SERVO_RIGHT];
  Xdot->ve[BFMS_OM_L] = -1./THAU * X->ve[BFMS_OM_L] - Kq * omega_square->ve[SERVO_LEFT] + Kv/THAU * u->ve[SERVO_LEFT];

}






#if 0
static void motor_model_derivative(VEC* x, VEC* u, VEC* xdot) {
  static VEC *temp1 = VNULL;
  static VEC *temp2 = VNULL;
  temp1 = v_resize(temp1,SERVOS_NB);
  temp2 = v_resize(temp2,SERVOS_NB);

  // omega_dot = -1/THAU*omega - Kq*omega^2 + Kv/THAU * V;
  temp1 = sv_mlt(-1./THAU, x, temp1);        /* temp1 = -1/THAU * x       */
  temp2 = v_star(x, x, temp2);               /* temp2 = x^2               */
  xdot = v_mltadd(temp1, temp2, -Kq, xdot);  /* xdot = temp1 - Kq*temp2   */
  xdot = v_mltadd(xdot, u, Kv/THAU, xdot);   /* xdot = xdot + Kv/THAU * u */
}
#endif
