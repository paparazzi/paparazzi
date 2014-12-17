#include "overo_sfb_controller.h"

#include "overo_estimator.h"
#include "std.h"
#include "stdio.h"
#include "stdlib.h"

#include "messages2.h"
#include "overo_gcs_com.h"

#define _CO (controller)

struct OveroController _CO;

#define GAIN (RadOfDeg(15))

static float z0 = 0, z1 = GAIN, z2 = 0, z3 = -GAIN, z4 = 0;
//static float x2=0, x3=-GAIN, x4=0;
static float x0 = GAIN, x1 = 0, x2 = -GAIN, x3 = 0, x4 = 0;

void control_send_messages(void)
{

  RunOnceEvery(15, {DOWNLINK_SEND_BETH_CONTROLLER_TWIST(gcs_com.udp_transport, &z0, &z1, &z2, &z3);});
}


void control_init(void)
{

  _CO.tilt_sp = 0.;
  _CO.elevation_sp = RadOfDeg(10);
  _CO.azimuth_sp = 0.;

  _CO.tilt_ref = 0.;
  _CO.elevation_ref = 0.;
  _CO.azimuth_ref = 0.;

  _CO.tilt_dot_ref = 0.;
  _CO.elevation_dot_ref = 0.;
  _CO.azimuth_dot_ref = 0.;

  _CO.cmd_sfb_pitch = 0.;
  _CO.cmd_sfb_thrust = 0.;

  _CO.cmd_df_pitch = 0.;
  _CO.cmd_df_thrust = 0.;

  _CO.cmd_pitch = 0.;
  _CO.cmd_thrust = 0.;

  _CO.a = 0.03;//theoretical=19.62
  _CO.a = 0.06;
  //_CO.b = 0.27;//theoretical=157.21
  _CO.b = 0.86;
  _CO.u_t_ref = 70;

  /*omegas - natural frequencies*/
  _CO.o_tilt = RadOfDeg(300);//was 100
  _CO.o_elev = RadOfDeg(100);
  _CO.o_azim = RadOfDeg(100);

  /*zetas - damping ratios*/
  _CO.z_tilt = 1.;
  _CO.z_elev = 1.;
  _CO.z_azim = 1.;

  _CO.armed = 0;
}



void control_run(void)
{

  static int foo = 0;

  calc_df_cmd();

  _CO.u_t_ref = _CO.cmd_df_thrust;

  calc_sfb_cmd();

  _CO.cmd_pitch = _CO.cmd_sfb_pitch + _CO.cmd_df_pitch;
  _CO.cmd_thrust = _CO.cmd_sfb_thrust + _CO.cmd_df_thrust;

  if (!(foo % 100)) {
    printf("P:%f T:%f \n", _CO.cmd_df_pitch, _CO.cmd_df_thrust);
  }
  foo++;

  Bound(_CO.cmd_thrust, 0, 100);
  Bound(_CO.cmd_pitch, -100, 100);

}

void calc_df_cmd(void)
{

  static uint32_t timecnt = 0;
  static float time = 0;

  const float dt = 1. / 512.;
  const float g = 9.8;
  const float freq1 = 1. / (2. * 3.14159);
  const float freq2 = 1. / (1. * 3.14159);
  const float const1 = 9.8 / 75.;
  const float const2 = 0.04;

  if (_CO.armed) {
    time = timecnt++ * dt;
  }

  /*  x2 = x2 + x3*dt;
    x3 = x3 + x4*dt;*/
  //x4 = GAIN*sin (2 * 3.14159 * freq2 * time);
  x0 = GAIN * cos(2 * 3.14159 * freq2 * time);
  x1 = -GAIN * sin(2 * 3.14159 * freq2 * time);
  x2 = -GAIN * cos(2 * 3.14159 * freq2 * time);
  x3 = GAIN * sin(2 * 3.14159 * freq2 * time);
  x4 = GAIN * cos(2 * 3.14159 * freq2 * time);

  /*  z0 = z0 + z1*dt ;
    z1 = z1 + z2*dt ;
    z2 = z2 + z3*dt ;
    z3 = z3 + z4*dt ;*/
  z0 = GAIN * sin(2 * 3.14159 * freq1 * time);
  z1 = GAIN * cos(2 * 3.14159 * freq1 * time);
  z2 = -GAIN * sin(2 * 3.14159 * freq1 * time);
  z3 = -GAIN * cos(2 * 3.14159 * freq1 * time);
  z4 = GAIN * sin(2 * 3.14159 * freq1 * time);

  _CO.cmd_df_thrust = (1 / const1) * sqrt(powf(x2, 2) + powf((z2 + g) , 2)) ;
  _CO.cmd_df_pitch  = (1 / const2) *
                      ((x4 * (z2 + 1) - z4 * x2) * (powf(z2 + g, 2) + powf(x2,
                          2)) - (2 * (z2 + g) * z3 + 2 * x2 * x3) * (x3 * (z2 + g) - z3 * x2)) /
                      powf((powf(z2 + g, 2) + powf(x2, 2)) , 2);

  Bound(_CO.cmd_df_thrust, 0, 100);
  Bound(_CO.cmd_df_pitch, -100, 100);
}

void calc_sfb_cmd(void)
{
  /*
   *  calculate errors
   */

  /*  const float err_tilt = estimator.tilt - _CO.tilt_ref;
    const float err_tilt_dot = estimator.tilt_dot - _CO.tilt_dot_ref;*/
  const float err_tilt = estimator.tilt - x0;
  const float err_tilt_dot = estimator.tilt_dot - x1;

  /*  const float err_elevation = estimator.elevation - _CO.elevation_ref;
    const float err_elevation_dot = estimator.elevation_dot - _CO.elevation_dot_ref;*/

  const float err_elevation = estimator.elevation - z0;
  const float err_elevation_dot = estimator.elevation_dot - z1;

  const float err_azimuth = estimator.azimuth - _CO.azimuth_ref;
  const float err_azimuth_dot = estimator.azimuth_dot - _CO.azimuth_dot_ref;

  /*
   *  Compute state feedback
   */

  _CO.cmd_sfb_pitch = -1 * (-1 *
                            err_azimuth
                            * (_CO.o_tilt * _CO.o_tilt * _CO.o_azim * _CO.o_azim * cos(estimator.tilt))
                            / (_CO.b * _CO.a * _CO.u_t_ref) +
                            err_elevation
                            * (_CO.o_tilt * _CO.o_tilt * _CO.o_elev * _CO.o_elev * sin(estimator.tilt))
                            / (_CO.b * _CO.a * _CO.u_t_ref) -
                            err_tilt
                            * (_CO.o_tilt * _CO.o_tilt) / (_CO.b) -    //+
                            err_azimuth_dot
                            * (_CO.o_tilt * _CO.o_tilt * 2 * _CO.z_azim * _CO.o_azim * cos(estimator.tilt))
                            / (_CO.b * _CO.a * _CO.u_t_ref) +
                            err_elevation_dot
                            * (_CO.o_tilt * _CO.o_tilt * 2 * _CO.z_elev * _CO.o_elev * sin(estimator.tilt))
                            / (_CO.b * _CO.a * _CO.u_t_ref) -
                            err_tilt_dot
                            * (2 * _CO.o_tilt * _CO.z_tilt)
                            / (_CO.b));

  _CO.cmd_sfb_thrust =
    err_azimuth           * _CO.o_azim * _CO.o_azim * sin(estimator.tilt) / _CO.a -
    err_elevation         * _CO.o_elev * _CO.o_elev * cos(estimator.tilt) / _CO.a +
    err_azimuth_dot   * 2 * _CO.z_azim * _CO.o_azim * sin(estimator.tilt) / _CO.a -
    err_elevation_dot * 2 * _CO.z_elev * _CO.o_elev * cos(estimator.tilt) / _CO.a  ;

  _CO.cmd_sfb_thrust = _CO.cmd_sfb_thrust * (1 / cos(estimator.elevation));

  //if (_CO.cmd_thrust<0.) _CO.cmd_thrust = 0;
  Bound(_CO.cmd_sfb_thrust, 0, 100);
  Bound(_CO.cmd_sfb_pitch, -100, 100);
}
