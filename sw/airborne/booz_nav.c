#include "booz_nav.h"

#include <math.h>

#include "booz_estimator.h"
#include "booz_control.h"
#include "radio_control.h"

#ifndef DISABLE_NAV
float booz_nav_horizontal_x_sp;
float booz_nav_horizontal_y_sp;
float booz_nav_horizontal_u_sp;
float booz_nav_horizontal_v_sp;
float booz_nav_horizontal_psi_sp;
float booz_nav_horizontal_pgain;
float booz_nav_horizontal_dgain;
float booz_nav_horizontal_speed_pre_gain;
float booz_nav_horizontal_max_pos_err;
float booz_nav_phi_command;
float booz_nav_theta_command;
#define BOOZ_NAV_HORIZONTAL_MAX_POS_ERR 10.
/* rad / m */
#define BOOZ_NAV_HORIZONTAL_PGAIN (RadOfDeg(20.) / BOOZ_NAV_HORIZONTAL_MAX_POS_ERR)
/* rad / (ms-1) */
#define BOOZ_NAV_HORIZONTAL_DGAIN (RadOfDeg(5.))
/* rad / (ms-1) */
#define BOOZ_NAV_HORIZONTAL_SPEED_PRE_GAIN (RadOfDeg(3.5))

#define BOOZ_NAV_HORIZONTAL_MAX_PHI_COMMAND   RadOfDeg(30.)
#define BOOZ_NAV_HORIZONTAL_MAX_THETA_COMMAND RadOfDeg(30.)

float booz_nav_vertical_z_sp;
float booz_nav_vertical_pgain;
float booz_nav_vertical_dgain;
#define BOOZ_NAV_VERTICAL_PGAIN 0.12
#define BOOZ_NAV_VERTICAL_DGAIN 0.15
#define BOOZ_NAV_VERT_HOVER_COMMAND 0.65
float booz_nav_power_command;
static void booz_nav_vertical_loop_run(void);
static void booz_nav_hovering_loop_run(void);
#endif


void booz_nav_init(void) {
#ifndef DISABLE_NAV
  booz_nav_horizontal_x_sp = 0.;
  booz_nav_horizontal_y_sp = 0.;
  booz_nav_horizontal_u_sp = 0.;
  booz_nav_horizontal_v_sp = 0.;
  booz_nav_horizontal_max_pos_err = BOOZ_NAV_HORIZONTAL_MAX_POS_ERR;
  booz_nav_horizontal_pgain = BOOZ_NAV_HORIZONTAL_PGAIN;
  booz_nav_horizontal_dgain = BOOZ_NAV_HORIZONTAL_DGAIN;
  booz_nav_horizontal_speed_pre_gain = BOOZ_NAV_HORIZONTAL_SPEED_PRE_GAIN; 
  booz_nav_vertical_pgain = BOOZ_NAV_VERTICAL_PGAIN;
  booz_nav_vertical_dgain = BOOZ_NAV_VERTICAL_DGAIN;
  booz_nav_vertical_z_sp = 0.;
  booz_nav_power_command = 0.;

#endif
}

void booz_nav_run(void) {
#ifndef DISABLE_NAV
  static uint8_t prescaler = 0;
  prescaler++;
  if (prescaler > 50) {
    prescaler = 0;
    booz_nav_vertical_loop_run();
    booz_nav_hovering_loop_run();
  }
  BoozControlAttitudeSetSetPoints(booz_nav_phi_command, booz_nav_theta_command, 
				  booz_nav_horizontal_psi_sp, booz_nav_power_command);
  booz_control_attitude_run();
#else
  /* on real ac, let nav kill all motoros */
  booz_control_commands[COMMAND_P] = 0;
  booz_control_commands[COMMAND_Q] = 0;
  booz_control_commands[COMMAND_R] = 0;
  booz_control_commands[COMMAND_THROTTLE] = 0;
#endif

}

#define STICK_ABSOLUTE_POS   0
#define STICK_ABSOLUTE_SPEED 1
#define STICK_RELATIVE_SPEED 2
#define STICK_MODE STICK_RELATIVE_SPEED
#define DT_READ_SETPOINTS 0.004
void booz_nav_read_setpoints_from_rc(void) {
#ifndef DISABLE_NAV
  booz_nav_vertical_z_sp    = -1. - 10. / MAX_PPRZ * (float)rc_values[RADIO_THROTTLE];
  booz_nav_horizontal_psi_sp = -RadOfDeg(30.) / MAX_PPRZ * (float)rc_values[RADIO_YAW];
#if defined STICK_MODE && STICK_MODE == STICK_ABSOLUTE_POS
  booz_nav_horizontal_x_sp = -20. / MAX_PPRZ * (float)rc_values[RADIO_PITCH]; /* +/- 20m */
  booz_nav_horizontal_y_sp = -20. / MAX_PPRZ * (float)rc_values[RADIO_ROLL];  /* warning RC roll negativ to the right !! +/- 20m */
  booz_nav_horizontal_u_sp = 0.;
  booz_nav_horizontal_v_sp = 0.;
#elif defined STICK_MODE && STICK_MODE == STICK_ABSOLUTE_SPEED
  booz_nav_horizontal_x_sp += -5. * DT_READ_SETPOINTS / MAX_PPRZ * (float)rc_values[RADIO_PITCH]; /* +/-5 m/s */
  booz_nav_horizontal_y_sp +=  5. * DT_READ_SETPOINTS / MAX_PPRZ * (float)rc_values[RADIO_ROLL];  /* +/-5 m/s */
  booz_nav_horizontal_u_sp = 0.;
  booz_nav_horizontal_v_sp = 0.;
#elif defined STICK_MODE && STICK_MODE == STICK_RELATIVE_SPEED
  booz_nav_horizontal_u_sp = -5. / MAX_PPRZ * (float)rc_values[RADIO_PITCH];
  booz_nav_horizontal_v_sp = -5. / MAX_PPRZ * (float)rc_values[RADIO_ROLL]; /* warning RC roll negativ to the right !! +/- 5m/s */
  /* convert vector to earth frame ( use transpose of DCM )  */
  float u_sp_earth = booz_nav_horizontal_u_sp * booz_estimator_dcm[AXIS_X][AXIS_X] +
                     booz_nav_horizontal_v_sp * booz_estimator_dcm[AXIS_Y][AXIS_X];
  float v_sp_earth = booz_nav_horizontal_u_sp * booz_estimator_dcm[AXIS_X][AXIS_Y] +
                     booz_nav_horizontal_v_sp * booz_estimator_dcm[AXIS_Y][AXIS_Y];
  booz_nav_horizontal_x_sp += (u_sp_earth * DT_READ_SETPOINTS);
  booz_nav_horizontal_y_sp += (v_sp_earth * DT_READ_SETPOINTS);
#endif /* STICK_MODE  */
#endif /* DISABLE_NAV */
}



#ifndef DISABLE_NAV

/* fisrt a vertical loop */ 
static void booz_nav_vertical_loop_run(void) {
  float vertical_err = booz_estimator_z - booz_nav_vertical_z_sp;
  Bound(vertical_err, -20., 20.);
  float bank_coef = 1. / fabs(cos(booz_estimator_phi)*cos(booz_estimator_theta));
  float adjusted_hover_power  = BOOZ_NAV_VERT_HOVER_COMMAND * bank_coef;
  //  printf("hover power command %f %f\n", bank_coef, adjusted_hover_power);
  booz_nav_power_command = adjusted_hover_power +
                           /* BOOZ_NAV_VERT_HOVER_COMMAND +  */
                           booz_nav_vertical_pgain * vertical_err +
                           booz_nav_vertical_dgain * booz_estimator_vz;

  Bound(booz_nav_power_command, 0., 1.);
}

/* now a horizontal hovering loop */
static void booz_nav_hovering_loop_run(void) {
  
  /* get a position error vector              */
  float x_error = booz_estimator_x - booz_nav_horizontal_x_sp;
  float y_error = booz_estimator_y - booz_nav_horizontal_y_sp;
  //  printf("earth pos error %f %f\n", x_error, y_error); 
  /* get norm and unit position error vector  */  
  float norm_error = sqrt(x_error*x_error+ y_error*y_error);
  float unit_x_error = x_error / norm_error;
  float unit_y_error = y_error / norm_error;
  //  printf("earth pos error %f %f %f\n", unit_x_error, unit_y_error, norm_error); 
  /* bound the error in amplitude             */
  Bound(norm_error, 0., booz_nav_horizontal_max_pos_err);
  /* convert unit error vector to body frame       */
  float x_unit_err_body = unit_x_error * booz_estimator_dcm[AXIS_X][AXIS_X] +
                          unit_y_error * booz_estimator_dcm[AXIS_X][AXIS_Y];
  float y_unit_err_body = unit_x_error * booz_estimator_dcm[AXIS_Y][AXIS_X] +
                          unit_y_error * booz_estimator_dcm[AXIS_Y][AXIS_Y];
  //  printf("body  pos error %f %f\n", x_unit_err_body, y_unit_err_body); 
  /* */

  booz_nav_phi_command   =  - booz_nav_horizontal_pgain * y_unit_err_body * norm_error +
                            - booz_nav_horizontal_dgain *  booz_estimator_v +
                              booz_nav_horizontal_speed_pre_gain * booz_nav_horizontal_v_sp;
  booz_nav_theta_command =    booz_nav_horizontal_pgain * x_unit_err_body * norm_error +
                              booz_nav_horizontal_dgain *  booz_estimator_u +
                            - booz_nav_horizontal_speed_pre_gain * booz_nav_horizontal_u_sp;
  
  Bound(booz_nav_phi_command, -BOOZ_NAV_HORIZONTAL_MAX_PHI_COMMAND, BOOZ_NAV_HORIZONTAL_MAX_PHI_COMMAND);
  Bound(booz_nav_theta_command, -BOOZ_NAV_HORIZONTAL_MAX_THETA_COMMAND, BOOZ_NAV_HORIZONTAL_MAX_THETA_COMMAND);

}
#endif
