#include "test/ahrs/ahrs_on_synth.h"

#include <stdio.h>

#include "subsystems/imu.h"
#include "subsystems/ahrs.h"
#include "subsystems/ahrs/ahrs_aligner.h"
#include "../simulator/nps/nps_random.h"

#include "../pprz_algebra_print.h"


char* ahrs_type_str[AHRS_TYPE_NB] = {
  "Int   Compl Euler",
  "Int   Compl Quat",
  "Float LKF   Quat",
  "Float Compl Rmat",
  "Float Compl Rmat 2",
  "Float Compl Quat" };

static void traj_static_static_init(void);
static void traj_static_static_update(void);

static void traj_step_phi_init(void);
static void traj_step_phi_update(void);

static void traj_step_phi_2nd_order_init(void);
static void traj_step_phi_2nd_order_update(void);

static void traj_step_biasp_init(void);
static void traj_step_biasp_update(void);

static void traj_static_sine_init(void);
static void traj_static_sine_update(void);

static void traj_sineX_quad_init(void);
static void traj_sineX_quad_update(void);

static void traj_coordinated_circle_init(void);
static void traj_coordinated_circle_update(void);

static void traj_stop_stop_x_init(void);
static void traj_stop_stop_x_update(void);

struct traj traj[] = {
  {.name="static", .desc="blaa",
   .init_fun=traj_static_static_init, .update_fun=traj_static_static_update},
  {.name="sine", .desc="blaa2",
   .init_fun=traj_static_sine_init, .update_fun=traj_static_sine_update},
  {.name="sineX", .desc="blaa2",
   .init_fun=traj_sineX_quad_init, .update_fun=traj_sineX_quad_update},
  {.name="step_phi", .desc="blaa2",
   .init_fun=traj_step_phi_init, .update_fun=traj_step_phi_update},
  {.name="step_phi2", .desc="blaa2",
   .init_fun=traj_step_phi_2nd_order_init, .update_fun=traj_step_phi_2nd_order_update},
  {.name="step_bias", .desc="blaa2",
   .init_fun=traj_step_biasp_init, .update_fun=traj_step_biasp_update},
  {.name="coordinated circle", .desc="blaa2",
   .init_fun=traj_coordinated_circle_init, .update_fun=traj_coordinated_circle_update},
  {.name="stop stop x", .desc="blaa2",
   .init_fun=traj_stop_stop_x_init, .update_fun=traj_stop_stop_x_update}
};


struct AhrsOnSynth aos;


void aos_init(int traj_nb) {

  aos.traj = &traj[traj_nb];

  aos.time = 0;
  aos.dt = 1./AHRS_PROPAGATE_FREQUENCY;
  aos.traj->ts = 0;
  aos.traj->ts = 1.; // default to one second

  /* default state */
  EULERS_ASSIGN(aos.ltp_to_imu_euler, RadOfDeg(0.), RadOfDeg(0.), RadOfDeg(0.));
  FLOAT_QUAT_OF_EULERS(aos.ltp_to_imu_quat, aos.ltp_to_imu_euler);
  RATES_ASSIGN(aos.imu_rates, RadOfDeg(0.), RadOfDeg(0.), RadOfDeg(0.));
  FLOAT_VECT3_ZERO(aos.ltp_pos);
  FLOAT_VECT3_ZERO(aos.ltp_vel);
  FLOAT_VECT3_ZERO(aos.ltp_accel);
  FLOAT_VECT3_ZERO(aos.ltp_jerk);
  aos.traj->init_fun();

  imu_init();
  ahrs_init();
  ahrs_aligner_init();

#ifdef PERFECT_SENSORS
  RATES_ASSIGN(aos.gyro_bias,  RadOfDeg(0.), RadOfDeg(0.), RadOfDeg(0.));
  RATES_ASSIGN(aos.gyro_noise, RadOfDeg(0.), RadOfDeg(0.), RadOfDeg(0.));
  VECT3_ASSIGN(aos.accel_noise, 0., 0., 0.);
#else
  RATES_ASSIGN(aos.gyro_bias,  RadOfDeg(1.), RadOfDeg(2.), RadOfDeg(3.));
  RATES_ASSIGN(aos.gyro_noise, RadOfDeg(1.), RadOfDeg(1.), RadOfDeg(1.));
  VECT3_ASSIGN(aos.accel_noise, .5, .5, .5);
#endif


#ifdef FORCE_ALIGNEMENT
  //  DISPLAY_FLOAT_QUAT_AS_EULERS_DEG("# oas quat", aos.ltp_to_imu_quat);
  aos_compute_sensors();
  //  DISPLAY_FLOAT_RATES_DEG("# oas gyro_bias", aos.gyro_bias);
  //  DISPLAY_FLOAT_RATES_DEG("# oas imu_rates", aos.imu_rates);
  VECT3_COPY(ahrs_aligner.lp_accel, imu.accel);
  VECT3_COPY(ahrs_aligner.lp_mag, imu.mag);
  RATES_COPY(ahrs_aligner.lp_gyro, imu.gyro);
  //  DISPLAY_INT32_RATES_AS_FLOAT_DEG("# ahrs_aligner.lp_gyro", ahrs_aligner.lp_gyro);
  ahrs_align();
  //  DISPLAY_FLOAT_RATES_DEG("# ahrs_impl.gyro_bias", ahrs_impl.gyro_bias);

#endif




#ifdef DISABLE_ALIGNEMENT
  printf("# DISABLE_ALIGNEMENT\n");
#endif
#ifdef DISABLE_PROPAGATE
  printf("# DISABLE_PROPAGATE\n");
#endif
#ifdef DISABLE_ACCEL_UPDATE
  printf("# DISABLE_ACCEL_UPDATE\n");
#endif
#ifdef DISABLE_MAG_UPDATE
  printf("# DISABLE_MAG_UPDATE\n");
#endif
  printf("# AHRS_TYPE  %s\n", ahrs_type_str[AHRS_TYPE]);
  printf("# AHRS_PROPAGATE_FREQUENCY %d\n", AHRS_PROPAGATE_FREQUENCY);
#ifdef AHRS_PROPAGATE_LOW_PASS_RATES
  printf("# AHRS_PROPAGATE_LOW_PASS_RATES\n");
#endif
#ifdef AHRS_MAG_UPDATE_YAW_ONLY
  printf("# AHRS_MAG_UPDATE_YAW_ONLY\n");
#endif
#ifdef AHRS_GRAVITY_UPDATE_COORDINATED_TURN
  printf("# AHRS_GRAVITY_UPDATE_COORDINATED_TURN\n");
#endif
#ifdef AHRS_GRAVITY_UPDATE_NORM_HEURISTIC
  printf("# AHRS_GRAVITY_UPDATE_NORM_HEURISTIC\n");
#endif
#ifdef PERFECT_SENSORS
  printf("# PERFECT_SENSORS\n");
#endif

  printf("# tajectory : %s\n", aos.traj->name);

};


void aos_compute_sensors(void) {

  struct FloatRates gyro;
  RATES_SUM(gyro, aos.imu_rates, aos.gyro_bias);
  //  printf("#aos.gyro_bias %f\n",DegOfRad( aos.gyro_bias.r));

  float_rates_add_gaussian_noise(&gyro, &aos.gyro_noise);

  RATES_BFP_OF_REAL(imu.gyro, gyro);
  RATES_BFP_OF_REAL(imu.gyro_prev, gyro);

  struct FloatVect3 g_ltp = {0., 0., 9.81};
  struct FloatVect3 accelero_ltp;
  VECT3_DIFF(accelero_ltp, aos.ltp_accel, g_ltp);
  struct FloatVect3 accelero_imu;
  FLOAT_QUAT_VMULT(accelero_imu, aos.ltp_to_imu_quat, accelero_ltp);

  float_vect3_add_gaussian_noise(&accelero_imu, &aos.accel_noise);
  ACCELS_BFP_OF_REAL(imu.accel, accelero_imu);


  struct FloatVect3 h_earth = {AHRS_H_X, AHRS_H_Y, AHRS_H_Z};
  struct FloatVect3 h_imu;
  FLOAT_QUAT_VMULT(h_imu, aos.ltp_to_imu_quat, h_earth);
  MAGS_BFP_OF_REAL(imu.mag, h_imu);

#ifdef AHRS_GRAVITY_UPDATE_COORDINATED_TURN
#if AHRS_TYPE == AHRS_TYPE_FCR2 || AHRS_TYPE == AHRS_TYPE_FCQ || AHRS_TYPE == AHRS_TYPE_FLQ
  VECT3_COPY(ahrs_impl.est_ltp_speed, aos.ltp_vel);
#endif
#if AHRS_TYPE == AHRS_TYPE_ICQ
  ahrs_impl.ltp_vel_norm = SPEED_BFP_OF_REAL(FLOAT_VECT3_NORM(aos.ltp_vel));
#endif
#endif


}

void aos_compute_state(void) {

  aos.time += aos.dt;
  aos.traj->update_fun();

}


void aos_run(void) {

  aos_compute_state();
  aos_compute_sensors();
#ifndef DISABLE_ALIGNEMENT
  if (ahrs.status == AHRS_UNINIT) {
    ahrs_aligner_run();
    if (ahrs_aligner.status == AHRS_ALIGNER_LOCKED)
      ahrs_align();
  }
  else {
#endif /* DISABLE_ALIGNEMENT */
    ahrs_propagate();
    ahrs_update_accel();
    ahrs_update_mag();
#ifndef DISABLE_ALIGNEMENT
  }
#endif

}




static void traj_static_static_init(void) {

  aos.traj->te = 120.;

}

static void traj_static_static_update(void) {

  //  if (aos.time > 3) {
  //    EULERS_ASSIGN(aos.ltp_to_imu_euler,   RadOfDeg(5), 0, 0);
  //    FLOAT_QUAT_OF_EULERS(aos.ltp_to_imu_quat, aos.ltp_to_imu_euler);
    //  }
  //  aos.imu_rates.p = 0.;
  //  aos.imu_rates.q = 0.;
  //  aos.imu_rates.r = 0.;

}


//
//
//
static void traj_static_sine_init(void) {

  aos.traj->te = 10.;

}

static void traj_static_sine_update(void) {


  aos.imu_rates.p = RadOfDeg(200)*cos(aos.time);
  aos.imu_rates.q = RadOfDeg(200)*cos(0.7*aos.time+2);
  aos.imu_rates.r = RadOfDeg(200)*cos(0.8*aos.time+1);
  FLOAT_QUAT_INTEGRATE(aos.ltp_to_imu_quat, aos.imu_rates, aos.dt);
  FLOAT_EULERS_OF_QUAT(aos.ltp_to_imu_euler, aos.ltp_to_imu_quat);

}


//
//  this is a sine trajectory along the x axis
//  we pretend a dragless vectoriel trust vehicle banks
//  to achieve it
//
static void traj_sineX_quad_init(void) {  aos.traj->te = 60.; }
static void traj_sineX_quad_update(void) {

  const float om = RadOfDeg(10);

  if ( aos.time > (M_PI/om) ) {
    const float a = 20;

    struct FloatVect3 jerk;
    VECT3_ASSIGN(jerk           , -a*om*om*om*cos(om*aos.time), 0, 0);
    VECT3_ASSIGN(aos.ltp_accel  , -a*om*om   *sin(om*aos.time), 0, 0);
    VECT3_ASSIGN(aos.ltp_vel    ,  a*om      *cos(om*aos.time), 0, 0);
    VECT3_ASSIGN(aos.ltp_pos    ,  a         *sin(om*aos.time), 0, 0);

    // this is based on differential flatness of the quad
    EULERS_ASSIGN(aos.ltp_to_imu_euler,    0., atan2(aos.ltp_accel.x, 9.81), 0.);
    FLOAT_QUAT_OF_EULERS(aos.ltp_to_imu_quat, aos.ltp_to_imu_euler);
    const struct FloatEulers e_dot = {
      0.,
      9.81*jerk.x / ( (9.81*9.81) + (aos.ltp_accel.x*aos.ltp_accel.x)),
      0. };
    FLOAT_RATES_OF_EULER_DOT(aos.imu_rates, aos.ltp_to_imu_euler, e_dot);
  }
}


//
//
//
static void traj_step_phi_init(void) { aos.traj->te = 40.;}
static void traj_step_phi_update(void) {
  if (aos.time > 5) {
    EULERS_ASSIGN(aos.ltp_to_imu_euler,   RadOfDeg(5), 0, 0);
    FLOAT_QUAT_OF_EULERS(aos.ltp_to_imu_quat, aos.ltp_to_imu_euler);
  }
}

//
//
//
static void traj_step_phi_2nd_order_init(void) {
  aos.traj->te = 0.;
  aos.traj->te = 40.;
}

static void traj_step_phi_2nd_order_update(void) {

  if (aos.time > 15) {
    const float omega = RadOfDeg(100);
    const float xi = 0.9;
    struct FloatRates raccel;
    RATES_ASSIGN(raccel, -2.*xi*omega*aos.imu_rates.p - omega*omega*(aos.ltp_to_imu_euler.phi - RadOfDeg(5)), 0., 0.);
    FLOAT_RATES_INTEGRATE_FI(aos.imu_rates, raccel, aos.dt);
    FLOAT_QUAT_INTEGRATE(aos.ltp_to_imu_quat, aos.imu_rates, aos.dt);
    FLOAT_EULERS_OF_QUAT(aos.ltp_to_imu_euler, aos.ltp_to_imu_quat);
  }

}


static void traj_step_biasp_init(void) { aos.traj->te = 120.; }
static void traj_step_biasp_update(void) {  if (aos.time > 5) aos.gyro_bias.p = RadOfDeg(3);}


static void traj_coordinated_circle_init(void) {

  aos.traj->te = 120.;

  const float speed = 15;  // m/s
  const float R = 100;     // radius in m
  // tan phi = v^2/Rg
  float phi = atan2(speed*speed, R*9.81);
  EULERS_ASSIGN(aos.ltp_to_imu_euler,   phi, 0, M_PI_2);
  FLOAT_QUAT_OF_EULERS(aos.ltp_to_imu_quat, aos.ltp_to_imu_euler);
}

static void traj_coordinated_circle_update(void) {
  const float speed = 15;  // m/s
  const float R = 100;     // radius in m
  float omega = speed / R;
  // tan phi = v^2/Rg
  float phi = atan2(speed*speed, R*9.81);
  if ( aos.time > 2.*M_PI/omega) {
    VECT3_ASSIGN(aos.ltp_pos,                R*cos(omega*aos.time),              R*sin(omega*aos.time), 0.);
    VECT3_ASSIGN(aos.ltp_vel,         -omega*R*sin(omega*aos.time),        omega*R*cos(omega*aos.time), 0.);
    VECT3_ASSIGN(aos.ltp_accel, -omega*omega*R*cos(omega*aos.time), -omega*omega*R*sin(omega*aos.time), 0.);


    //  float psi = atan2(aos.ltp_pos.y, aos.ltp_pos.x);
    float psi = M_PI_2 + omega*aos.time; while (psi>M_PI) psi -= 2.*M_PI;
    EULERS_ASSIGN(aos.ltp_to_imu_euler,   phi, 0, psi);
    FLOAT_QUAT_OF_EULERS(aos.ltp_to_imu_quat, aos.ltp_to_imu_euler);

    struct FloatEulers e_dot;
    EULERS_ASSIGN(e_dot, 0., 0., omega);
    FLOAT_RATES_OF_EULER_DOT(aos.imu_rates, aos.ltp_to_imu_euler, e_dot);
  }

}



//static char** traj_stop_stop_x_desc(void) {
  //  static const char** desc =
  //    {"stop top", NULL};
//  return  desc;
//}
static void  traj_stop_stop_x_init(void) { aos.traj->te = 30.;}

static void  traj_stop_stop_x_update(void){

  const float t0 = 5.;
  const float dt_jerk = 0.75;
  const float dt_nojerk = 10.;
  const float val_jerk = 5.;

  FLOAT_VECT3_INTEGRATE_FI(aos.ltp_pos, aos.ltp_vel, aos.dt);
  FLOAT_VECT3_INTEGRATE_FI(aos.ltp_vel, aos.ltp_accel, aos.dt);
  FLOAT_VECT3_INTEGRATE_FI(aos.ltp_accel, aos.ltp_jerk, aos.dt);

  if (aos.time < t0) return;
  else if (aos.time < t0+dt_jerk) {
    VECT3_ASSIGN(aos.ltp_jerk           ,  val_jerk, 0., 0.); }
  else if (aos.time < t0 + 2.*dt_jerk) {
    VECT3_ASSIGN(aos.ltp_jerk           , -val_jerk, 0., 0.); }
  else if (aos.time < t0 + 2.*dt_jerk + dt_nojerk) {
    VECT3_ASSIGN(aos.ltp_jerk           ,       0. , 0., 0.); }
  else if (aos.time < t0 + 3.*dt_jerk + dt_nojerk) {
    VECT3_ASSIGN(aos.ltp_jerk           , -val_jerk, 0., 0.); }
  else if (aos.time < t0 + 4.*dt_jerk + dt_nojerk) {
    VECT3_ASSIGN(aos.ltp_jerk           ,  val_jerk, 0., 0.); }
  else {
    VECT3_ASSIGN(aos.ltp_jerk           ,       0. , 0., 0.); }


  // this is based on differential flatness of the quad
  EULERS_ASSIGN(aos.ltp_to_imu_euler,    0., atan2(aos.ltp_accel.x, 9.81), 0.);
  FLOAT_QUAT_OF_EULERS(aos.ltp_to_imu_quat, aos.ltp_to_imu_euler);
  const struct FloatEulers e_dot = {
    0.,
    9.81*aos.ltp_jerk.x / ( (9.81*9.81) + (aos.ltp_accel.x*aos.ltp_accel.x)),
    0. };
  FLOAT_RATES_OF_EULER_DOT(aos.imu_rates, aos.ltp_to_imu_euler, e_dot);

}
