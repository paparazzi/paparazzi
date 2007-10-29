#include "booz_nav.h"

#include "booz_estimator.h"
#include "booz_control.h"
#include "radio_control.h"

#ifndef DISABLE_NAV
float booz_nav_horizontal_x_sp;
float booz_nav_horizontal_y_sp;
float booz_nav_horizontal_pgain;
float booz_nav_horizontal_dgain;
float booz_nav_phi_command;
float booz_nav_theta_command;
#define BOOZ_NAV_HORIZONTAL_MAX_POS_ERR 10.

float booz_nav_vertical_z_sp;
float booz_nav_vertical_pgain;
float booz_nav_vertical_dgain;
#define BOOZ_NAV_VERTICAL_PGAIN 0.1
#define BOOZ_NAV_VERTICAL_DGAIN 0.1
#define BOOZ_NAV_VERT_HOVER_COMMAND 0.68
float booz_nav_power_command;
static void booz_nav_vertical_loop_run(void);
static void booz_nav_hovering_loop_run(void);
#endif


void booz_nav_init(void) {
#ifndef DISABLE_NAV
  booz_nav_vertical_pgain = BOOZ_NAV_VERTICAL_PGAIN;
  booz_nav_vertical_dgain = BOOZ_NAV_VERTICAL_DGAIN;
  booz_nav_vertical_z_sp = -10.;
  booz_nav_power_command = 0.; //BOOZ_NAV_VERT_HOVER_COMMAND; 

#endif
}

void booz_nav_run(void) {
#ifndef DISABLE_NAV
  booz_nav_vertical_loop_run();
  booz_nav_hovering_loop_run();
  BoozControlAttitudeSetSetPoints(booz_nav_phi_command, booz_nav_theta_command, 0., booz_nav_power_command);
  booz_control_attitude_run();
#else
  /* on real ac, let nav kill all motoros */
  booz_control_commands[COMMAND_P] = 0;
  booz_control_commands[COMMAND_Q] = 0;
  booz_control_commands[COMMAND_R] = 0;
  booz_control_commands[COMMAND_THROTTLE] = 0;
#endif

}

void booz_nav_read_setpoints_from_rc(void) {
#ifndef DISABLE_NAV
  booz_nav_vertical_z_sp = -0 - 10. / MAX_PPRZ * (float)rc_values[RADIO_THROTTLE];
  booz_nav_horizontal_x_sp += 0.01 / MAX_PPRZ * (float)rc_values[RADIO_PITCH];
  booz_nav_horizontal_y_sp += 0.01 / MAX_PPRZ * (float)rc_values[RADIO_ROLL];
#endif
}



#ifndef DISABLE_NAV

/* fisrt a vertical loop */ 
static void booz_nav_vertical_loop_run(void) {
  float vertical_err = booz_estimator_z - booz_nav_vertical_z_sp;
  Bound(vertical_err, -20., 20.);
  booz_nav_power_command = BOOZ_NAV_VERT_HOVER_COMMAND  + 
                           booz_nav_vertical_pgain * vertical_err +
                           booz_nav_vertical_pgain * booz_estimator_vz ;
}

/* now a horizontal hovering loop */
static void booz_nav_hovering_loop_run(void) {
  
  /* get a position error vector */
  float x_error = booz_estimator_x - booz_nav_horizontal_x_sp;
  float y_error = booz_estimator_y - booz_nav_horizontal_y_sp;
  /* get norm and unit position error vector */  
  float norm_error = sqrt(x_error*x_error+ y_error*y_error);
  float unit_x_error = x_error / norm_error;
  float unit_y_error = y_error / norm_error;
  Bound(norm_error, 0., BOOZ_NAV_HORIZONTAL_MAX_POS_ERR);
  /* convert error vector to body frame */
  float x_err_body = x_error * booz_estimator_dcm[AXIS_X][AXIS_X] +
                     y_error * booz_estimator_dcm[AXIS_X][AXIS_Y];
  float y_err_body = x_error * booz_estimator_dcm[AXIS_Y][AXIS_X] +
                     y_error * booz_estimator_dcm[AXIS_Y][AXIS_Y];

  


  booz_nav_phi_command = booz_nav_horizontal_pgain * x_err_body;
  booz_nav_theta_command = booz_nav_horizontal_pgain * y_err_body;

}
#endif
