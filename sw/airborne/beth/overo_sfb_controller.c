#include "overo_sfb_controller.h"

#include "overo_estimator.h"
#include "std.h"
#include "stdio.h"
#include "stdlib.h"

#include "messages2.h"
#include "overo_gcs_com.h"

#define _CO (controller)

struct OveroController _CO;

void control_send_messages(void) {

  RunOnceEvery(15, {DOWNLINK_SEND_BETH_CONTROLLER_SFB(gcs_com.udp_transport,
			&_CO.tilt_sp);});

}


void control_init(void) {

  _CO.tilt_sp = 0.;
  _CO.elevation_sp = RadOfDeg(10);
  _CO.azimuth_sp = 0.;

  _CO.tilt_ref = 0.;
  _CO.elevation_ref = 0;
  _CO.azimuth_ref = 0.;

  _CO.tilt_dot_ref = 0.;
  _CO.elevation_dot_ref = 0;
  _CO.azimuth_dot_ref = 0.;

  _CO.cmd_pitch = 0.;
  _CO.cmd_thrust = 0.;
  
  _CO.a = 0.03;
  _CO.b = 0.27;
  _CO.u_t_ref = 40;
  
  /*omegas - natural frequencies*/
  _CO.o_tilt = RadOfDeg(100);
  _CO.o_elev = RadOfDeg(100);
  _CO.o_azim = RadOfDeg(100);

  /*zetas - damping ratios*/
  _CO.z_tilt = 1;
  _CO.z_elev = 1;
  _CO.z_azim = 1;  

  _CO.armed = 0;
}



void control_run(void) {

  static int foo=0;

  /*
   *  calculate errors
   */

  const float err_tilt = estimator.tilt - _CO.tilt_ref;
  const float err_tilt_dot = estimator.tilt_dot - _CO.tilt_dot_ref;

  const float err_elevation = estimator.elevation - _CO.elevation_ref;
  const float err_elevation_dot = estimator.elevation_dot - _CO.elevation_dot_ref;

  const float err_azimuth = estimator.azimuth - _CO.azimuth_ref;
  const float err_azimuth_dot = estimator.azimuth_dot - _CO.azimuth_dot_ref;

  /*
   *  Compute state feedback
   */

  _CO.cmd_pitch = -1*( -1*
    err_azimuth   
      * ( _CO.o_tilt * _CO.o_tilt * _CO.o_azim * _CO.o_azim * cos(estimator.tilt) ) 
      / ( _CO.b * _CO.a * _CO.u_t_ref) +
    err_elevation    
      * ( _CO.o_tilt * _CO.o_tilt * _CO.o_elev * _CO.o_elev * sin(estimator.tilt) ) 
      / ( _CO.b * _CO.a * _CO.u_t_ref) -
    err_tilt         
      * ( _CO.o_tilt * _CO.o_tilt ) / ( _CO.b ) -//+
    err_azimuth_dot  
      * ( _CO.o_tilt * _CO.o_tilt * 2 * _CO.z_azim * _CO.o_azim * cos(estimator.tilt) ) 
      / ( _CO.b * _CO.a * _CO.u_t_ref) +
    err_elevation_dot 
      * ( _CO.o_tilt * _CO.o_tilt * 2 * _CO.z_elev * _CO.o_elev * sin(estimator.tilt) ) 
      / ( _CO.b * _CO.a * _CO.u_t_ref) -
    err_tilt_dot 
      * ( 2 * _CO.o_tilt * _CO.z_tilt ) 
      / ( _CO.b ) );
 
  _CO.cmd_thrust = 
    err_azimuth           * _CO.o_azim * _CO.o_azim * sin(estimator.tilt) / _CO.a - 
    err_elevation         * _CO.o_elev * _CO.o_elev * cos(estimator.tilt) / _CO.a + 
    err_azimuth_dot   * 2 * _CO.z_azim * _CO.o_azim * sin(estimator.tilt) / _CO.a -
    err_elevation_dot * 2 * _CO.z_elev * _CO.o_elev * cos(estimator.tilt) / _CO.a  ;

  _CO.cmd_thrust = _CO.cmd_thrust*(1/cos(estimator.elevation));

  //if (_CO.cmd_thrust<0.) _CO.cmd_thrust = 0;
  Bound(_CO.cmd_thrust,0,100);
  Bound(_CO.cmd_pitch,-100,100);

  if (!(foo%100)) {
    printf("P:%f T:%f \n",_CO.cmd_pitch,_CO.cmd_thrust);
  }
  foo++; 

}

