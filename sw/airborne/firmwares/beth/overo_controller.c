#include "overo_controller.h"

#include "overo_estimator.h"
#include "std.h"
#include "stdio.h"

#include "messages2.h"
#include "overo_gcs_com.h"

struct OveroController controller;

void control_init(void)
{
//  controller.kp = 0.05;
//  controller.kd = 0.01;

  controller.tilt_sp = 0.;
  controller.elevation_sp = RadOfDeg(10);
  controller.azimuth_sp = 0.;

  controller.omega_tilt_ref = RadOfDeg(600);
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
}

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
}

void control_run(void)
{

  /*
   *  propagate reference
   */
  const float dt_ctl = 1. / 512.;
  const float thrust_constant = 40.;

  controller.tilt_ref = controller.tilt_ref + controller.tilt_dot_ref * dt_ctl;
  controller.tilt_dot_ref = controller.tilt_dot_ref + controller.tilt_ddot_ref * dt_ctl;
  controller.tilt_ddot_ref = -2 * controller.omega_tilt_ref * controller.xi_ref * controller.tilt_dot_ref
                             - controller.omega_tilt_ref * controller.omega_tilt_ref * (controller.tilt_ref - controller.tilt_sp);

  controller.elevation_ref = controller.elevation_ref + controller.elevation_dot_ref * dt_ctl;
  controller.elevation_dot_ref = controller.elevation_dot_ref + controller.elevation_ddot_ref * dt_ctl;
  controller.elevation_ddot_ref = -2 * controller.omega_elevation_ref * controller.xi_ref * controller.elevation_dot_ref
                                  - controller.omega_elevation_ref * controller.omega_elevation_ref * (controller.elevation_ref -
                                      controller.elevation_sp);

  controller.azimuth_ref = controller.azimuth_ref + controller.azimuth_dot_ref * dt_ctl;
  controller.azimuth_dot_ref = controller.azimuth_dot_ref + controller.azimuth_ddot_ref * dt_ctl;
  controller.azimuth_ddot_ref = -2 * controller.omega_azimuth_ref * controller.xi_ref * controller.azimuth_dot_ref
                                - controller.omega_azimuth_ref * controller.omega_azimuth_ref * (controller.azimuth_ref - controller.azimuth_sp);

  static int foo = 0;

  /*
   *  calculate errors
   */

  const float err_tilt = estimator.tilt - controller.tilt_ref;
  const float err_tilt_dot = estimator.tilt_dot - controller.tilt_dot_ref;

  const float err_elevation = estimator.elevation - controller.elevation_ref;
  const float err_elevation_dot = estimator.elevation_dot - controller.elevation_dot_ref;

  const float err_azimuth = estimator.azimuth - controller.azimuth_ref;
  const float err_azimuth_dot = estimator.azimuth_dot - controller.azimuth_dot_ref;

  /*
   *  Compute feedforward and feedback commands
   */

  controller.cmd_pitch_ff = controller.one_over_J * controller.tilt_ddot_ref;
  controller.cmd_pitch_fb = controller.one_over_J * (2 * controller.xi_cl * controller.omega_cl * err_tilt_dot) +
                            controller.one_over_J * (controller.omega_cl * controller.omega_cl * err_tilt);

  controller.cmd_thrust_ff = controller.mass * controller.elevation_ddot_ref;
  controller.cmd_thrust_fb = -controller.mass * (2 * controller.xi_cl * controller.omega_cl * err_elevation_dot) -
                             controller.mass * (controller.omega_cl * controller.omega_cl * err_elevation);

  controller.cmd_azimuth_ff = controller.one_over_J * controller.azimuth_ddot_ref;
  controller.cmd_azimuth_fb = controller.one_over_J * (2 * controller.xi_cl * controller.omega_cl * err_azimuth_dot) +
                              controller.one_over_J * (controller.omega_cl * controller.omega_cl * err_azimuth);

  controller.cmd_pitch =  controller.cmd_pitch_ff + controller.cmd_pitch_fb;// +
  controller.tilt_sp =
    controller.azim_gain * (-controller.cmd_azimuth_fb); //+ controller.cmd_azimuth_ff);
  controller.cmd_thrust = controller.cmd_thrust_ff + controller.cmd_thrust_fb + thrust_constant;
  controller.cmd_thrust = controller.cmd_thrust * (1 / cos(estimator.elevation));

  //if (controller.cmd_thrust<0.) controller.cmd_thrust = 0;
  Bound(controller.cmd_thrust, 0, 100);
  Bound(controller.cmd_pitch, -100, 100);

  if (!(foo % 100)) {
    //printf("pitch : ff:%f fb:%f (%f)\n",controller.cmd_pitch_ff, controller.cmd_pitch_fb,estimator.tilt_dot);
    //printf("thrust: ff:%f fb:%f (%f %f)\n",controller.cmd_thrust_ff, controller.cmd_thrust_fb,estimator.elevation,estimator.elevation_dot);
    //printf("%f %f %f\n",controller.tilt_ref,controller.tilt_dot_ref,controller.tilt_ddot_ref);
  }
  foo++;

}

