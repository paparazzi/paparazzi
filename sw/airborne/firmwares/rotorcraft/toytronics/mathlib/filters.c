/*
 * This file is part of mathlib.
 *
 * Copyright (C) 2010-2011 Greg Horn <ghorn@stanford.edu>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/*
 * filters.c
 */

#include <math.h>
#include <stdio.h>

#include "filters.h"
#include "misc_math.h"
#include "xyz.h"

void
simple_lowpass(double dt, double tau, double *state, double input)
{
  double emdt = exp(-dt/tau);
  *state = (*state)*emdt + input*(1-emdt);
}

void
xyz_simple_lowpass(double dt, double tau, xyz_t * state, const xyz_t * const input)
{
  simple_lowpass(dt, tau, &(state->x), input->x);
  simple_lowpass(dt, tau, &(state->y), input->y);
  simple_lowpass(dt, tau, &(state->z), input->z);
}

double
simple_pid(const double y,
           const double y_dot,
           const double r,
           const double r_dot,
           const double dt,
           double *i_state,
           const pid_conf_t * const c)
{
  // warn if gains are negative
  if (c->kp < 0.0) printf("warning: negative p gain in simple_pid\n");
  if (c->ki < 0.0) printf("warning: negative i gain in simple_pid\n");
  if (c->kd < 0.0) printf("warning: negative d gain in simple_pid\n");
  if (c->integral_bound < 0.0) printf("warning: negative integral state bound simple_pid\n");

  // errors
  double err = r - y;
  double err_dot = r_dot - y_dot;

  // integral state
  *i_state += err*dt;
  BOUND( *i_state, -c->integral_bound, c->integral_bound);

  // apply pid
  return c->kp * (err + c->ki * (*i_state) + c->kd * err_dot);
};
