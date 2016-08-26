/*
 * Copyright (C) 2006  Pascal Brisset, Antoine Drouin, Michel Gorraz
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
 */

/**
 *  @file firmwares/fixedwing/guidance/guidance_v.c
 *  Vertical control using total energy control for fixed wing vehicles.
 *


	=================================================
	Energy:
	------
	E 		= mgh + 1/2mV^2
	Edot / V 	= (gamma + Vdot/g) * W

	equilibrium

	Vdot / g = Thrust/W - Drag/W - sin(gamma)
	with: Drag/Weight = (Cl/Cd)^-1

	-glide angle: Vdot = 0, T=0 ==> gamma = Cd/Cl
	-level flight: Vdot = 0, gamma=0 ==> W/T = Cl/Cd
	=================================================

	Strategy:  thrust = path + acceleration[g] (total energy)
		   pitch = path - acceleration[g]  (energy balance)

	Pseudo-Control Unit = dimensionless acceleration [g]

		- pitch <-> pseudocontrol:    sin(Theta) steers Vdot in [g]
		- throttle <-> pseudocontrol: motor characteristic as function of V x throttle steeds VDot

 */

#pragma message "CAUTION! Using TOTAL ENERGY CONTROLLER: Experimental!"

#include "firmwares/fixedwing/guidance/energy_ctrl.h"
#include "estimator.h"
#include "subsystems/nav.h"
#include "generated/airframe.h"
#include "firmwares/fixedwing/autopilot.h"
#include "subsystems/ahrs.h"
#include "subsystems/imu.h"

/////// DEFAULT GUIDANCE_V NECESSITIES //////

/* mode */
uint8_t v_ctl_mode = V_CTL_MODE_MANUAL;
uint8_t v_ctl_climb_mode = V_CTL_CLIMB_MODE_AUTO_THROTTLE;
uint8_t v_ctl_auto_throttle_submode = V_CTL_CLIMB_MODE_AUTO_THROTTLE;
float v_ctl_auto_throttle_sum_err = 0;
float v_ctl_auto_airspeed_controlled = 0;
float v_ctl_auto_groundspeed_setpoint = 0;

#ifdef LOITER_TRIM
#error "Energy Controller can not accep Loiter Trim"
#endif
//#ifdef V_CTL_AUTO_THROTTLE_MIN_CRUISE_THROTTLE
//#error

/////// ACTUALLY USED STUFF //////

/* outer loop */
float v_ctl_altitude_setpoint;
float v_ctl_altitude_pre_climb; ///< Path Angle
float v_ctl_altitude_pgain;
float v_ctl_airspeed_pgain;
float v_ctl_altitude_error;    ///< in meters, (setpoint - alt) -> positive = too low

float v_ctl_auto_airspeed_setpoint; ///< in meters per second

float v_ctl_max_climb = 2;
float v_ctl_max_acceleration = 0.5;

/* inner loop */
float v_ctl_climb_setpoint;

/* "auto throttle" inner loop parameters */
float v_ctl_desired_acceleration;

float v_ctl_auto_throttle_cruise_throttle;
float v_ctl_auto_throttle_nominal_cruise_throttle;
float v_ctl_auto_throttle_nominal_cruise_pitch;
float v_ctl_auto_throttle_climb_throttle_increment;
float v_ctl_auto_throttle_pitch_of_vz_pgain;

float v_ctl_auto_throttle_of_airspeed_pgain;
float v_ctl_auto_throttle_of_airspeed_igain;
float v_ctl_auto_pitch_of_airspeed_pgain;
float v_ctl_auto_pitch_of_airspeed_igain;
float v_ctl_auto_pitch_of_airspeed_dgain;

float v_ctl_energy_total_pgain;
float v_ctl_energy_total_igain;

float v_ctl_energy_diff_pgain;
float v_ctl_energy_diff_igain;


pprz_t v_ctl_throttle_setpoint;
pprz_t v_ctl_throttle_slewed;


/////////////////////////////////////////////////
// Automatically found airplane characteristics

float ac_char_climb_pitch = 0.0f;
float ac_char_climb_max = 0.0f;
int ac_char_climb_count = 0;
float ac_char_descend_pitch = 0.0f;
float ac_char_descend_max = 0.0f;
int ac_char_descend_count = 0;
float ac_char_cruise_throttle = 0.0f;
float ac_char_cruise_pitch = 0.0f;
int ac_char_cruise_count = 0;

static void ac_char_average(float* last, float new, int count)
{
  *last = (((*last) * (((float)count) - 1.0f)) + new) / ((float) count);
}

static void ac_char_update(float throttle, float pitch, float climb, float accelerate)
{
  if ((accelerate > -0.02) && (accelerate < 0.02))
  {
    if (throttle >= 1.0f)
    {
      ac_char_climb_count++;
      ac_char_average(&ac_char_climb_pitch, pitch * 57.6f,            ac_char_climb_count );
      ac_char_average(&ac_char_climb_max ,  estimator_z_dot,  ac_char_climb_count );
    }
    else if (throttle <= 0.0f)
    {
      ac_char_descend_count++;
      ac_char_average(&ac_char_descend_pitch, pitch * 57.6f ,           ac_char_descend_count );
      ac_char_average(&ac_char_descend_max ,  estimator_z_dot , ac_char_descend_count );
    }
    else if ((climb > -0.125) && (climb < 0.125))
    {
      ac_char_cruise_count++;
      ac_char_average(&ac_char_cruise_throttle , throttle , ac_char_cruise_count );
      ac_char_average(&ac_char_cruise_pitch    , pitch * 57.6f  ,   ac_char_cruise_count );
    }
  }
}


void v_ctl_init( void ) {
  /* mode */
  v_ctl_mode = V_CTL_MODE_MANUAL;

  /* outer loop */
  v_ctl_altitude_setpoint = 0.;
  v_ctl_altitude_pgain = V_CTL_ALTITUDE_PGAIN;
  v_ctl_auto_airspeed_setpoint = NOMINAL_AIRSPEED;

  /* inner loops */
  v_ctl_climb_setpoint = 0.;

  /* "auto throttle" inner loop parameters */
  v_ctl_auto_throttle_nominal_cruise_throttle = V_CTL_AUTO_THROTTLE_NOMINAL_CRUISE_THROTTLE;
  v_ctl_auto_throttle_climb_throttle_increment = V_CTL_AUTO_THROTTLE_CLIMB_THROTTLE_INCREMENT;
  v_ctl_auto_throttle_pitch_of_vz_pgain = V_CTL_AUTO_THROTTLE_PITCH_OF_VZ_PGAIN;

  v_ctl_throttle_setpoint = 0;
}

/**
 * outer loop
 * \brief Computes v_ctl_climb_setpoint and sets v_ctl_auto_throttle_submode
 */

void v_ctl_altitude_loop( void )
{
  // Imput Checks
  if (v_ctl_auto_airspeed_setpoint <= 0.0f) v_ctl_auto_airspeed_setpoint = NOMINAL_AIRSPEED;

  // Altitude Controller
  v_ctl_altitude_error = v_ctl_altitude_setpoint - estimator_z;
  float sp = v_ctl_altitude_pgain * v_ctl_altitude_error + v_ctl_altitude_pre_climb ;
  BoundAbs(sp, v_ctl_max_climb);

  float incr = sp - v_ctl_climb_setpoint;
  BoundAbs(incr, 2.0 / 4.0);
  v_ctl_climb_setpoint += incr;
}


/**
 * auto throttle inner loop
 * \brief
 */

const float dt = 0.01f;

float lp_vdot[5];

static float low_pass_vdot(float v);
static float low_pass_vdot(float v)
{
  lp_vdot[4] += (v - lp_vdot[4]) / 3;
  lp_vdot[3] += (lp_vdot[4] - lp_vdot[3]) / 3;
  lp_vdot[2] += (lp_vdot[3] - lp_vdot[2]) / 3;
  lp_vdot[1] += (lp_vdot[2] - lp_vdot[1]) / 3;
  lp_vdot[0] += (lp_vdot[1] - lp_vdot[0]) / 3;

  return lp_vdot[0];
}

void v_ctl_climb_loop( void )
{
  // Airspeed outerloop: positive means we need to accelerate
  float speed_error = v_ctl_auto_airspeed_setpoint - estimator_airspeed;

  // Speed Controller to PseudoControl: gain 1 -> 5m/s error = 0.5g acceleration
  v_ctl_desired_acceleration = speed_error * v_ctl_airspeed_pgain / 9.81f;
  BoundAbs(v_ctl_desired_acceleration, v_ctl_max_acceleration);

  // Actual Acceleration from IMU: attempt to reconstruct the actual kinematic acceleration
#ifndef SITL
  struct FloatVect3 accel_float = {0,0,0};
  ACCELS_FLOAT_OF_BFP(accel_float, imu.accel);
  float vdot = ( accel_float.x / 9.81f - sin(ahrs_float.ltp_to_imu_euler.theta) );
#else
  float vdot = 0;
#endif

  // Acceleration Error: positive means UAV needs to accelerate: needs extra energy
  float vdot_err = low_pass_vdot( v_ctl_desired_acceleration - vdot );

  // Flight Path Outerloop: positive means needs to climb more: needs extra energy
  float gamma_err  = (v_ctl_climb_setpoint - estimator_z_dot) / v_ctl_auto_airspeed_setpoint;

  // Total Energy Error: positive means energy should be added
  float en_tot_err = gamma_err + vdot_err;

  // Energy Distribution Error: positive means energy should go from overspeed to altitude = pitch up
  float en_dis_err = gamma_err - vdot_err;

  // Auto Cruise Throttle
  if (launch && (v_ctl_mode >= V_CTL_MODE_AUTO_CLIMB))
  {
    v_ctl_auto_throttle_nominal_cruise_throttle +=
        	  v_ctl_auto_throttle_of_airspeed_igain * speed_error * dt
		+ en_tot_err * v_ctl_energy_total_igain * dt;
    if (v_ctl_auto_throttle_nominal_cruise_throttle < 0.0f) v_ctl_auto_throttle_nominal_cruise_throttle = 0.0f;
    else if (v_ctl_auto_throttle_nominal_cruise_throttle > 1.0f) v_ctl_auto_throttle_nominal_cruise_throttle = 1.0f;
  }

  // Total Controller
  float controlled_throttle = v_ctl_auto_throttle_nominal_cruise_throttle
    + v_ctl_auto_throttle_climb_throttle_increment * v_ctl_climb_setpoint
    + v_ctl_auto_throttle_of_airspeed_pgain * speed_error
    + v_ctl_energy_total_pgain * en_tot_err;


  if ((controlled_throttle >= 1.0f) || (controlled_throttle <= 0.0f))
  {
    // If your energy supply is not sufficient, then neglect the climb requirement
    en_dis_err = -vdot_err;
  }

  /* pitch pre-command */
  if (launch && (v_ctl_mode >= V_CTL_MODE_AUTO_CLIMB))
  {
    v_ctl_auto_throttle_nominal_cruise_pitch +=  v_ctl_auto_pitch_of_airspeed_igain * (-speed_error) * dt
                          + v_ctl_energy_diff_igain * en_dis_err * dt;
    Bound(v_ctl_auto_throttle_nominal_cruise_pitch,H_CTL_PITCH_MIN_SETPOINT, H_CTL_PITCH_MAX_SETPOINT);
  }
  float v_ctl_pitch_of_vz =
		+ (v_ctl_climb_setpoint /*+ d_err * v_ctl_auto_throttle_pitch_of_vz_dgain*/) * v_ctl_auto_throttle_pitch_of_vz_pgain
		- v_ctl_auto_pitch_of_airspeed_pgain * speed_error
                + v_ctl_auto_pitch_of_airspeed_dgain * vdot
                + v_ctl_energy_diff_pgain * en_dis_err
                + v_ctl_auto_throttle_nominal_cruise_pitch;

  nav_pitch = v_ctl_pitch_of_vz;

  ac_char_update(controlled_throttle, v_ctl_pitch_of_vz, v_ctl_climb_setpoint, v_ctl_desired_acceleration);

  v_ctl_throttle_setpoint = TRIM_UPPRZ(controlled_throttle * MAX_PPRZ);
}


#ifdef V_CTL_THROTTLE_SLEW_LIMITER
#define V_CTL_THROTTLE_SLEW (1./CONTROL_RATE/(V_CTL_THROTTLE_SLEW_LIMITER))
#endif

#ifndef V_CTL_THROTTLE_SLEW
#define V_CTL_THROTTLE_SLEW 1.
#endif

/** \brief Computes slewed throttle from throttle setpoint
    called at 20Hz
 */
void v_ctl_throttle_slew( void ) {
  pprz_t diff_throttle = v_ctl_throttle_setpoint - v_ctl_throttle_slewed;
  BoundAbs(diff_throttle, TRIM_PPRZ(V_CTL_THROTTLE_SLEW*MAX_PPRZ));
  v_ctl_throttle_slewed += diff_throttle;
}
