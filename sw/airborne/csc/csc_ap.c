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

/** \file csc_ap.c
 */

#include "csc_ap.h"

#include <inttypes.h>
#include "commands.h"
#include "csc_xsens.h"
#include "led.h"
#include "pprz_algebra_float.h"
#include "string.h"
#include "radio_control.h"

struct control_gains csc_gains;
struct control_reference csc_reference;
struct control_reference csc_errors;
struct control_trims csc_trims;

static const int xsens_id = 0;

void csc_ap_init( void )
{
  csc_gains.roll_kp = 3500;
  csc_gains.pitch_kp = 0;
  csc_gains.yaw_kp = 8000;
  csc_gains.roll_kd = 1000;
  csc_gains.pitch_kd = 0;
  csc_gains.yaw_kd = 2500;
  
  memset(&csc_reference, 0, sizeof(struct control_reference));
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
  errors->eulers.psi = xsens_eulers.psi - csc_reference.eulers.psi;

  errors->rates.p = xsens_rates.p - csc_reference.rates.p;
  errors->rates.q = xsens_rates.q - csc_reference.rates.q;
  errors->rates.r = xsens_rates.r - csc_reference.rates.r;
}

static void calculate_reference(struct control_reference *reference)
{
  reference->eulers.phi = M_PI / 6.0 * rc_values[RADIO_ROLL];
  reference->eulers.theta = M_PI  / 6.0 * rc_values[RADIO_PITCH];
  reference->eulers.psi = M_PI  / 6.0 * rc_values[RADIO_YAW];

  reference->eulers.phi /= MAX_PPRZ;
  reference->eulers.theta /= MAX_PPRZ;
  reference->eulers.psi /= MAX_PPRZ;
}

void csc_ap_periodic( void )
{
  static int counter = 0;
  calculate_reference(&csc_reference);
  calculate_errors(&csc_errors);

  commands[COMMAND_ROLL] = -csc_gains.roll_kp * csc_errors.eulers.phi
			   + csc_gains.roll_kd * csc_errors.rates.p;
  commands[COMMAND_ROLL] += csc_trims.aileron;

  commands[COMMAND_PITCH] = -csc_gains.pitch_kp * csc_errors.eulers.theta
			   + csc_gains.pitch_kd * csc_errors.rates.q;
  commands[COMMAND_PITCH] += csc_trims.elevator;

  commands[COMMAND_YAW] = csc_gains.yaw_kp * csc_errors.eulers.psi
			   + csc_gains.yaw_kd * csc_errors.rates.r;
  commands[COMMAND_YAW] += csc_trims.rudder;
}
