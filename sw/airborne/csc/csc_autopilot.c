/*
 * Copyright (C) 2009 Joby Energy
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA. 
 *
 */

/** \file csc_autopilot.c
 */

#include "csc_autopilot.h"

#include <inttypes.h>
#include <math.h>

#include "commands.h"
#include "csc_xsens.h"
#include "led.h"
#include "math/pprz_algebra_float.h"
#include "string.h"
#include "radio_control.h"
#include "pwm_input.h"
#include "LPC21xx.h"
#include "print.h"

struct control_gains csc_gains;
struct control_reference csc_reference;
struct control_reference csc_errors;
struct control_trims csc_trims;
struct control_gains csc_gamma;
float csc_vane_angle;
float csc_vane_angle_offset = 180;

static const int xsens_id = 0;
float csc_yaw_rudder;
float csc_yaw_aileron;
float csc_yaw_deadband;
float csc_yaw_setpoint_rate;
float csc_yaw_setpoint_range;
float csc_vane_weight;
float csc_vane_filter_constant;

float csc_drag_pitch = 0;
float csc_drag_yaw = 0;

#define PWM_INPUT_COUNTS_PER_REV 61358.
static void update_vane_angle( void )
{
  csc_vane_angle =  (csc_vane_filter_constant * csc_vane_angle) + (1-csc_vane_filter_constant)*RadOfDeg( 360. * pwm_input_duration / PWM_INPUT_COUNTS_PER_REV) - RadOfDeg(csc_vane_angle_offset);
}

void csc_ap_init( void )
{
  csc_gains.roll_kp = 4218;
  csc_gains.roll_kd = 1000;
  csc_gains.roll_ki = 50;
  csc_gains.pitch_kp = 0;
  csc_gains.pitch_kd = 0;
  csc_gains.pitch_ki = 0;
  csc_gains.yaw_kp = 5000;
  csc_gains.yaw_kd = 800;
  csc_gains.yaw_ki = 10;
  csc_yaw_rudder = 0.33;
  csc_yaw_aileron = 1.00;
  csc_yaw_deadband = 1.00;
  csc_vane_weight = 0.1;

  csc_trims.elevator = 2020;
  csc_trims.aileron = -80;
  csc_trims.rudder = 80;

  csc_gamma.roll_kp = 0;
  csc_gamma.roll_kd = 0;
  csc_gamma.roll_ki = 0;
  csc_gamma.pitch_kp = 0;
  csc_gamma.pitch_kd = 0;
  csc_gamma.pitch_ki = 0;
  csc_gamma.yaw_kp = 0;
  csc_gamma.yaw_kd = 0;
  csc_gamma.yaw_ki = 0;
  
  memset(&csc_reference, 0, sizeof(struct control_reference));
}

static void update_ki(float new_ki, float *ki, float *ierror)
{
  if (new_ki == 0) {
    *ierror = 0;
  } else {
    *ierror *= *ki / new_ki;
  }
  *ki = new_ki;
}

#define I_CMD_BOUND 0.4

static void bound_ierror(float igain, float *ierror)
{
  float cmd = *ierror * igain;
  if (cmd > (MAX_PPRZ * I_CMD_BOUND)) {
    *ierror = MAX_PPRZ * I_CMD_BOUND / igain;
  } else if (cmd < (MIN_PPRZ * I_CMD_BOUND)) {
    *ierror = MIN_PPRZ * I_CMD_BOUND / igain;
  }
}

void csc_autopilot_set_roll_ki(float ki)
{
  update_ki(ki, &csc_gains.roll_ki, &csc_errors.eulers_i.phi);
}

void csc_autopilot_set_pitch_ki(float ki)
{
  update_ki(ki, &csc_gains.pitch_ki, &csc_errors.eulers_i.theta);
}

void csc_autopilot_set_yaw_ki(float ki)
{
  update_ki(ki, &csc_gains.yaw_ki, &csc_errors.eulers_i.psi);
}

static void calculate_errors(struct control_reference *errors)
{
  struct FloatEulers xsens_eulers;
  struct FloatRates xsens_rates;

  xsens_eulers.phi = xsens_phi[xsens_id];
  xsens_eulers.theta = xsens_theta[xsens_id];
  xsens_eulers.psi =  xsens_psi[xsens_id];

  xsens_rates.p = xsens_gyro_x[xsens_id];
  xsens_rates.q = xsens_gyro_y[xsens_id];
  xsens_rates.r = xsens_gyro_z[xsens_id];

  errors->eulers.phi = xsens_eulers.phi - csc_reference.eulers.phi;
  errors->eulers.theta = xsens_eulers.theta - csc_reference.eulers.theta;
  //errors->eulers.psi = xsens_eulers.psi - csc_reference.eulers.psi;
  //errors->eulers.psi = csc_vane_angle - csc_reference.eulers.psi;
  errors->eulers.psi = (csc_vane_angle*csc_vane_weight) 
    + (xsens_eulers.psi - csc_reference.eulers.psi)*(1 - csc_vane_weight);

  errors->rates.p = xsens_rates.p - csc_reference.rates.p;
  errors->rates.q = xsens_rates.q - csc_reference.rates.q;
  errors->rates.r = xsens_rates.r - csc_reference.rates.r;

  errors->eulers_i.phi += xsens_eulers.phi;
  errors->eulers_i.theta += xsens_eulers.theta;
  errors->eulers_i.psi += xsens_eulers.psi;

  /* Deadband in yaw -- prevents it going nuts around an angle */
  float yaw_deadband = RadOfDeg(csc_yaw_deadband);

  if (errors->eulers.psi <= yaw_deadband && errors->eulers.psi >= -yaw_deadband) {
    errors->eulers.psi = 0;
  } else if (errors->eulers.psi <= yaw_deadband*2 && errors->eulers.psi >= -yaw_deadband*2) {
    if (errors->eulers.psi < 0) {
      errors->eulers.psi = (errors->eulers.psi + yaw_deadband)*2;
    } else 
      errors->eulers.psi = (errors->eulers.psi - yaw_deadband)*2;
  }
  
  bound_ierror(csc_gains.roll_ki, &errors->eulers_i.phi);
  bound_ierror(csc_gains.pitch_ki, &errors->eulers_i.theta);
  bound_ierror(csc_gains.yaw_ki, &errors->eulers_i.psi);
}

static void calculate_reference(struct control_reference *reference, int time)
{
  reference->eulers.psi = M_PI  / 6.0 * rc_values[RADIO_YAW] + 100*csc_yaw_setpoint_range*sin(0.01*time*csc_yaw_setpoint_rate);
  reference->eulers.theta = M_PI  / 6.0 * rc_values[RADIO_PITCH];
  reference->eulers.phi = M_PI / 6.0 * rc_values[RADIO_ROLL];

  reference->eulers.phi /= MAX_PPRZ;
  reference->eulers.theta /= MAX_PPRZ;
  reference->eulers.psi /= MAX_PPRZ;
}

void csc_ap_periodic(int time)
{
  //static int counter = 0;
  update_vane_angle();
  calculate_reference(&csc_reference, time);
  calculate_errors(&csc_errors);

  commands[COMMAND_ROLL] = -csc_gains.roll_kp * (csc_errors.eulers.phi)
                          + csc_gains.roll_kd * (csc_errors.rates.p)
                          - csc_gains.roll_ki * (csc_errors.eulers_i.phi);
  commands[COMMAND_ROLL] += csc_trims.aileron;

  commands[COMMAND_PITCH] = -csc_gains.pitch_kp * csc_errors.eulers.theta
			   + csc_gains.pitch_kd * csc_errors.rates.q
			   - csc_gains.pitch_ki * csc_errors.eulers_i.theta;
  commands[COMMAND_PITCH] += csc_trims.elevator;

  commands[COMMAND_ROLL] += (-csc_gains.yaw_kp * csc_errors.eulers.psi
			   - csc_gains.yaw_kd * csc_errors.rates.r
			     - csc_gains.yaw_ki * csc_errors.eulers_i.psi) * csc_yaw_aileron;

  commands[COMMAND_YAW] = csc_trims.rudder;

  commands[COMMAND_YAW] += (-csc_gains.yaw_kp * csc_errors.eulers.psi
			   - csc_gains.yaw_kd * csc_errors.rates.r
			    - csc_gains.yaw_ki * csc_errors.eulers_i.psi) * csc_yaw_rudder;

  csc_ap_update_gains(&csc_errors, &csc_gains);
}

void csc_ap_clear_ierrors( void )
{
  csc_errors.eulers_i.phi = 0;
  csc_errors.eulers_i.theta = 0;
  csc_errors.eulers_i.psi = 0;
}

void csc_ap_set_trims( void )
{
  csc_trims.aileron = commands[COMMAND_ROLL];
  csc_trims.elevator = commands[COMMAND_PITCH];
  csc_trims.rudder = commands[COMMAND_YAW];
}

void csc_ap_update_gains(struct control_reference *errors, struct control_gains *gains)
{
  // Adaptation law based on state error to be controlled, rate and i error
  gains->pitch_kp += -csc_gamma.pitch_kp*errors->eulers.theta*fabs(errors->eulers.theta);
  gains->pitch_kd += -csc_gamma.pitch_kd*errors->eulers.theta*xsens_gyro_y[xsens_id];
  gains->pitch_ki += -csc_gamma.pitch_ki*errors->eulers.theta*errors->eulers_i.theta;
  gains->roll_kp += -csc_gamma.roll_kp*errors->eulers.phi*fabs(errors->eulers.phi);
  gains->roll_kd += -csc_gamma.roll_kd*errors->eulers.phi*xsens_gyro_x[xsens_id];
  gains->roll_ki += -csc_gamma.roll_ki*errors->eulers.phi*errors->eulers_i.phi;
  gains->yaw_kp += -csc_gamma.yaw_kp*errors->eulers.psi*fabs(errors->eulers.psi);
  gains->yaw_kd += -csc_gamma.yaw_kd*errors->eulers.psi*xsens_gyro_z[xsens_id];
  gains->yaw_ki += -csc_gamma.yaw_ki*errors->eulers.psi*errors->eulers_i.psi;
}
