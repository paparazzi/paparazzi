#include "overo_twist_controller.h"

#include "overo_estimator.h"
#include "std.h"
#include "stdio.h"
#include "stdlib.h" //for abs()

#include "messages2.h"
#include "overo_gcs_com.h"

struct OveroTwistController controller;

void control_send_messages(void)
{

  RunOnceEvery(15, {DOWNLINK_SEND_BETH_CONTROLLER(gcs_com.udp_transport,
                    &controller.cmd_pitch, &controller.cmd_thrust,
                    &controller.cmd_pitch_ff, &controller.cmd_pitch_fb,
                    &controller.cmd_thrust_ff, &controller.cmd_thrust_fb,
                    &controller.tilt_sp, &controller.tilt_ref, &controller.tilt_dot_ref,
                    &controller.elevation_sp, &controller.elevation_ref, &controller.elevation_dot_ref,
                    &controller.azimuth_sp);
                   });

  RunOnceEvery(15, {DOWNLINK_SEND_BETH_CONTROLLER_TWIST(gcs_com.udp_transport,
                    &controller.S[1], &controller.S_dot, &controller.U_twt[1], &controller.error);
                   });
}


void control_init(void)
{

  printf("Twisting controller initializing\n");

  controller.tilt_sp = 0.;
  controller.elevation_sp = RadOfDeg(10);
  controller.azimuth_sp = 0.;

// controller.omega_tilt_ref = RadOfDeg(600);
  controller.omega_tilt_ref = RadOfDeg(1200);
  controller.omega_elevation_ref = RadOfDeg(120);
  controller.omega_azimuth_ref = RadOfDeg(60);
  controller.xi_ref = 1.;

  controller.tilt_ref = estimator.tilt;
  controller.tilt_dot_ref = estimator.tilt_dot;
  controller.tilt_ddot_ref = 0.;

  //controller.elevation_ref = estimator.elevation;
  controller.elevation_ref = RadOfDeg(-28);
  controller.elevation_dot_ref = estimator.elevation_dot;
  controller.elevation_ddot_ref = 0.;

  controller.azimuth_ref = estimator.azimuth;
  controller.azimuth_dot_ref = 0.;
  controller.azimuth_ddot_ref = 0.;

  controller.one_over_J = 2.;
  controller.mass = 5.;
  controller.azim_gain = 0.005;

  controller.omega_cl = RadOfDeg(600);
  controller.xi_cl = 1.;

  controller.cmd_pitch_ff = 0.;
  controller.cmd_pitch_fb = 0.;

  controller.cmd_thrust_ff = 0.;
  controller.cmd_thrust_fb = 0.;

  controller.cmd_pitch = 0.;
  controller.cmd_thrust = 0.;

  controller.armed = 0;

  /***** Coef twisting ****/
  controller.ulim = 1.0;
  controller.Vm = 0.1; //should this now be 1/512?
  controller.VM = 300.0;

  controller.S[1] = 0.0;
  controller.S[0] = 0.0;

  controller.U_twt[1] = 0.0;
  controller.U_twt[0] = 0.0;

  controller.satval1 = 0.087;
  controller.satval2 = 0.141;

  controller.c = 4.5;
  controller.error = 0;

  printf("Vm=%f VM=%f satval1=%f satval2=%f c=%f\n", controller.Vm, controller.VM, controller.satval1, controller.satval2,
         controller.c);
}



void control_run(void)
{
  static int foo = 0;
  /*
   *  propagate reference
   */
  const float dt_ctl = 1. / 512.;
  const float thrust_constant = 40.;

  //make setpoint a cosine ; for testing
  float new_sp = controller.tilt_sp * cos(6.28 / 5.*foo / 512.);

  controller.tilt_ref = controller.tilt_ref + controller.tilt_dot_ref * dt_ctl;
  controller.tilt_dot_ref = controller.tilt_dot_ref + controller.tilt_ddot_ref * dt_ctl;
  controller.tilt_ddot_ref = -2 * controller.omega_tilt_ref * controller.xi_ref * controller.tilt_dot_ref
                             - controller.omega_tilt_ref * controller.omega_tilt_ref * (controller.tilt_ref - new_sp);

  controller.elevation_ref = controller.elevation_ref + controller.elevation_dot_ref * dt_ctl;
  controller.elevation_dot_ref = controller.elevation_dot_ref + controller.elevation_ddot_ref * dt_ctl;
  controller.elevation_ddot_ref = -2 * controller.omega_elevation_ref * controller.xi_ref * controller.elevation_dot_ref
                                  - controller.omega_elevation_ref * controller.omega_elevation_ref * (controller.elevation_ref -
                                      controller.elevation_sp);

#if USE_AZIMUTH
  controller.azimuth_ref = controller.azimuth_ref + controller.azimuth_dot_ref * dt_ctl;
  controller.azimuth_dot_ref = controller.azimuth_dot_ref + controller.azimuth_ddot_ref * dt_ctl;
  controller.azimuth_ddot_ref = -2 * controller.omega_azimuth_ref * controller.xi_ref * controller.azimuth_dot_ref
                                - controller.omega_azimuth_ref * controller.omega_azimuth_ref * (controller.azimuth_ref - controller.azimuth_sp);
#endif


  /*
   *  calculate errors
   */

  /*  const float err_tilt = estimator.tilt - controller.tilt_ref;
    const float err_tilt_dot = estimator.tilt_dot - controller.tilt_dot_ref;*/

  const float err_elevation = estimator.elevation - controller.elevation_ref;
  const float err_elevation_dot = estimator.elevation_dot - controller.elevation_dot_ref;

#if USE_AZIMUTH
  const float err_azimuth = estimator.azimuth - controller.azimuth_ref;
  const float err_azimuth_dot = estimator.azimuth_dot - controller.azimuth_dot_ref;
#endif

  /*
   *  Compute feedforward and feedback commands
   */

  controller.cmd_pitch_ff = controller.one_over_J * controller.tilt_ddot_ref;

  /*  controller.cmd_pitch_fb = controller.one_over_J * (2 * controller.xi_cl * controller.omega_cl * err_tilt_dot) +
          controller.one_over_J * (controller.omega_cl * controller.omega_cl * err_tilt);*/

  controller.cmd_pitch_fb = get_U_twt();

  controller.cmd_thrust_ff = controller.mass * controller.elevation_ddot_ref;
  controller.cmd_thrust_fb = -controller.mass * (2 * controller.xi_cl * controller.omega_cl * err_elevation_dot) -
                             controller.mass * (controller.omega_cl * controller.omega_cl * err_elevation);

#if USE_AZIMUTH
  controller.cmd_azimuth_ff = controller.one_over_J * controller.azimuth_ddot_ref;
  controller.cmd_azimuth_fb = controller.one_over_J * (2 * controller.xi_cl * controller.omega_cl * err_azimuth_dot) +
                              controller.one_over_J * (controller.omega_cl * controller.omega_cl * err_azimuth);
#endif

  controller.cmd_pitch =  /*controller.cmd_pitch_ff*/ + controller.cmd_pitch_fb;

#if USE_AZIMUTH
  controller.tilt_sp = controller.azim_gain * (-controller.cmd_azimuth_fb);
#endif

  controller.cmd_thrust = get_U_twt2();
//  controller.cmd_thrust = controller.cmd_thrust_ff + controller.cmd_thrust_fb + thrust_constant;
  //not needed as twisting doesn't care about this nonlinearity
//  controller.cmd_thrust = controller.cmd_thrust*(1/cos(estimator.elevation));

  //if (controller.cmd_thrust<0.) controller.cmd_thrust = 0;
  Bound(controller.cmd_thrust, 0, 100);
  Bound(controller.cmd_pitch, -100, 100);

  if (!(foo % 128)) {
    //printf("pitch : ff:%f fb:%f (%f)\n",controller.cmd_pitch_ff, controller.cmd_pitch_fb,estimator.tilt_dot);
    //printf("thrust: ff:%f fb:%f (%f %f)\n",controller.cmd_thrust_ff, controller.cmd_thrust_fb,estimator.elevation,estimator.elevation_dot);
    //printf("%f %f %f\n",controller.tilt_ref,controller.tilt_dot_ref,controller.tilt_ddot_ref);
    //printf("t: %f\n",controller.cmd_thrust);
  }
  foo++;

}

//TODO: replace both functions with one that takes parameters (or a structure of params)

/*Fonction qui obtient la commande twisiting à appliquer chaque periode*/
float get_U_twt()
{

  /**Definition des constantes du modèle**/
  const float Gain = -65;
  const float Te = 1 / 512.;

  /**Variables utilisés par la loi de commande**/
  static volatile float yd[2] = {0.0, 0.0};
  static volatile float y[2] = {0., 0.};
  //static float emax = 0.035;    // en rad, initialement 2

  /**Variables pour l'algorithme**/
  float udot;
  float sens;

  //Acquisition consigne
  yd[1] = controller.tilt_ref;
  //Acquisition mesure
  y[1] = estimator.tilt;

  /***************************/

  /**Calcul Surface et derive Surface**/
  // S[1],y[1],yd[1] new value
  // S[0],y[0],yd[0] last value

  //gain K=Te
  //controller.S[1] = (double)( ( (1+controller.c) * (y[1]-yd[1]) - (y[0]-yd[0]) )  ) ;
  //controller.S[1] = (double)( ( (1+controller.c) * (y[1]-yd[1]) - estimator.tilt_dot ) * 0.8 ) ;
  controller.S[1] = (float)(controller.c * (y[1] - yd[1]) + estimator.tilt_dot - controller.tilt_dot_ref);
  controller.S_dot = (controller.S[1] - controller.S[0]);
  /*************************************/

  //On va dire que si l'erreur est d'un valeur inferieur a emax, on applique la commande anterieure
  /*  if ( abs(y[1] - yd[1]) < emax ) {
      U_twt[1] = U_twt[0];
    } else {*/
  /**Algorithme twisting**/
  if (controller.S[1] < 0.0) {
    sens = -1.0;
  } else {
    sens = 1.0;
  }

  if (abs(controller.U_twt[1]) < controller.ulim) {
    if ((controller.S[1] * controller.S_dot) > 0) {
      udot = -controller.VM * sens;
    } else {
      udot = -controller.Vm * sens;
    }
  } else {
    udot = -controller.U_twt[1];
  }

  // Integration de u, qu'avec 2 valeurs, penser à faire plus
  // u[1] new , u[0] old
  controller.U_twt[1] = controller.U_twt[0] + (Te * udot);
  //}
  /**********************/

  /**Saturation de l'integrateur**/

  if ((controller.S[1] > -controller.satval1) && (controller.S[1] < controller.satval1)) {
    Bound(controller.U_twt[1], -controller.satval1, controller.satval1);
  } else {
    Bound(controller.U_twt[1], -controller.satval2, controller.satval2);
  }
  /********************************/

  /**Mises à jour**/
  controller.U_twt[0] = controller.U_twt[1];
  yd[0] = yd[1];
  y[0] = y[1];

  controller.S[0] = controller.S[1];

  return Gain * controller.U_twt[1];

}

float get_U_twt2()
{

  /**Definition des constantes du modèle**/
  const float Gain = 800.;
  const float Te = 1 / 512.;

  /**Variables utilisés par la loi de commande**/
  static volatile float yd2[2] = {0.0, 0.0};
  static volatile float y2[2] = {0., 0.};
  //static float emax = 0.035;    // en rad, initialement 2

  /**Variables pour l'algorithme**/
  float udot2;
  float sens2;
  static float S2[2] = {0.0, 0.0};
  static float S2_dot = 0;
  static float U2_twt[2] = {0.0, 0.0};

  //Acquisition consigne
  yd2[1] = controller.elevation_ref;
  //Acquisition mesure
  y2[1] = estimator.elevation;

  /***************************/

  /**Calcul Surface et derive Surface**/
  // S[1],y[1],yd[1] new value
  // S[0],y[0],yd[0] last value

  //gain K=Te
  //controller.S[1] = (double)( ( (1+controller.c) * (y[1]-yd[1]) - (y[0]-yd[0]) )  ) ;
  //controller.S[1] = (double)( ( (1+controller.c) * (y[1]-yd[1]) - estimator.tilt_dot ) * 0.8 ) ;
  S2[1] = (float)(controller.c * (y2[1] - yd2[1]) + estimator.elevation_dot - controller.elevation_dot_ref);
  S2_dot = (S2[1] - S2[0]);
  /*************************************/

  //On va dire que si l'erreur est d'un valeur inferieur a emax, on applique la commande anterieure
  /*  if ( abs(y[1] - yd[1]) < emax ) {
      U_twt[1] = U_twt[0];
    } else {*/
  /**Algorithme twisting**/
  if (S2[1] < 0.0) {
    sens2 = -1.0;
  } else {
    sens2 = 1.0;
  }

  if (abs(U2_twt[1]) < controller.ulim) {
    if ((S2[1] * S2_dot) > 0) {
      udot2 = -controller.VM * sens2;
    } else {
      udot2 = -controller.Vm * sens2;
    }
  } else {
    udot2 = -U2_twt[1];
  }

  // Integration de u, qu'avec 2 valeurs, penser à faire plus
  // u[1] new , u[0] old
  U2_twt[1] = U2_twt[0] + (Te * udot2);
  //}
  /**********************/

  /**Saturation de l'integrateur**/

  if ((S2[1] > -controller.satval1) && (S2[1] < controller.satval1)) {
    Bound(U2_twt[1], -controller.satval1, controller.satval1);
  } else {
    Bound(U2_twt[1], -controller.satval2, controller.satval2);
  }
  /********************************/

  /**Mises à jour**/
  U2_twt[0] = U2_twt[1];
  yd2[0] = yd2[1];
  y2[0] = y2[1];

  S2[0] = S2[1];

#define NUMSAMPS (1000)
  float retval = Gain * U2_twt[1];

  static int sum = 0;
  static int i = 0;

  sum = sum + retval;

  if (i == (NUMSAMPS - 1)) {
    i = 0;
    printf("avg: %f\n", sum / (float)NUMSAMPS);
    sum = 0;
  } else {
    i++;
  }

  return retval;

}
