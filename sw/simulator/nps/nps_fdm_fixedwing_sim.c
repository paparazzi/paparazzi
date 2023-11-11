/*
 * Copyright (C) 2023 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "nps_fdm.h"

#include <stdlib.h>
#include <stdio.h>

#include "std.h"
#include "math/pprz_geodetic.h"
#include "math/pprz_geodetic_double.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_isa.h"

#include "generated/airframe.h"
#include "generated/flight_plan.h"

#include "state.h"

// Check if rover firmware
#ifndef FIXEDWING_FIRMWARE
#error "The module nps_fdm_fixedwing_sim is a basic flight model for fixedwing only"
#endif

#ifndef ROLL_RESPONSE_FACTOR
#define ROLL_RESPONSE_FACTOR 15.
#endif

#ifndef PITCH_RESPONSE_FACTOR
#define PITCH_RESPONSE_FACTOR 1.
#endif

#ifndef YAW_RESPONSE_FACTOR
#define YAW_RESPONSE_FACTOR 1.
#endif

#ifndef WEIGHT
#define WEIGHT 1.
#endif

#ifndef ROLL_MAX_SETPOINT
#define ROLL_MAX_SETPOINT RadOfDeg(40.)
#endif

#ifndef ROLL_RATE_MAX_SETPOINT
#define ROLL_RATE_MAX_SETPOINT RadOfDeg(15.)
#endif

#ifndef NOMINAL_AIRSPEED
#error "Please define NOMINAL_AIRSPEED in your airframe file"
#endif

#ifndef MAXIMUM_AIRSPEED
#define MAXIMUM_AIRSPEED (NOMINAL_AIRSPEED * 1.5)
#endif

#ifndef MAXIMUM_POWER
#define MAXIMUM_POWER (5. * MAXIMUM_AIRSPEED * WEIGHT)
#endif

#ifndef AUTO_THROTTLE_NOMINAL_CRUISE_THROTTLE
#define AUTO_THROTTLE_NOMINAL_CRUISE_THROTTLE 0.45
#endif

// NpsFdm structure
struct NpsFdm fdm;

// Reference point
static struct LtpDef_d ltpdef;

// Static functions declaration
static void init_ltp(void);

struct fixedwing_sim_state {
  struct EnuCoor_d pos;
  struct EnuCoor_d speed;
  struct DoubleEulers attitude;
  struct DoubleEulers rates;
  double airspeed;
  double delta_roll;
  double delta_pitch;
  double delta_thrust;
  struct EnuCoor_d wind;
};

static struct fixedwing_sim_state sim_state;


/** NPS FDM rover init ***************************/
void nps_fdm_init(double dt)
{
  fdm.init_dt = dt; // (1 / simulation freq)
  fdm.curr_dt = dt;
  fdm.time = dt;

  fdm.on_ground = TRUE;

  fdm.nan_count = 0;
  fdm.pressure = -1;
  fdm.pressure_sl = PPRZ_ISA_SEA_LEVEL_PRESSURE;
  fdm.total_pressure = -1;
  fdm.dynamic_pressure = -1;
  fdm.temperature = -1;

  fdm.ltpprz_to_body_eulers.phi = 0.0;
  fdm.ltpprz_to_body_eulers.theta = 0.0;
  fdm.ltpprz_to_body_eulers.psi = 0.0;

  VECT3_ASSIGN(sim_state.pos, 0., 0., 0.);
  VECT3_ASSIGN(sim_state.speed, 0., 0., 0.);
  EULERS_ASSIGN(sim_state.attitude, 0., 0., 0.);
  EULERS_ASSIGN(sim_state.rates, 0., 0., 0.);
  VECT3_ASSIGN(sim_state.wind, 0., 0., 0.);
  sim_state.airspeed = 0.;
  sim_state.delta_roll = 0.;
  sim_state.delta_pitch = 0.;
  sim_state.delta_thrust = 0.;

  init_ltp();
}


/** Minimum complexity flight dynamic model
 *  In legacy Paparazzi simulator, was implemented in OCaml
 *  and correspond to the 'sim' target
 *
 * Johnson, E.N., Fontaine, S.G., and Kahn, A.D.,
 * “Minimum Complexity Uninhabited Air Vehicle Guidance And Flight Control System,”
 * Proceedings of the 20th Digital Avionics Systems Conference, 2001.
 * http://www.ae.gatech.edu/~ejohnson/JohnsonFontaineKahn.pdf

 * Johnson, E.N. and Fontaine, S.G.,
 * “Use Of Flight Simulation To Complement Flight Testing Of Low-Cost UAVs,”
 * Proceedings of the AIAA Modeling and Simulation Technology Conference, 2001.
 * http://www.ae.gatech.edu/~ejohnson/AIAA%202001-4059.pdf
 */
void nps_fdm_run_step(bool launch, double *commands, int commands_nb __attribute__((unused)))
{

  // commands
  sim_state.delta_roll = commands[COMMAND_ROLL] * MAX_PPRZ;
  sim_state.delta_pitch = -commands[COMMAND_PITCH] * MAX_PPRZ; // invert back to correct sign
  sim_state.delta_thrust = commands[COMMAND_THROTTLE];

  static bool already_launched = false;
  if (launch && !already_launched) {
    sim_state.airspeed = NOMINAL_AIRSPEED;
    already_launched = true;
  }
  if (sim_state.airspeed == 0. && sim_state.delta_thrust > 0.) {
    sim_state.airspeed = NOMINAL_AIRSPEED;
  }

  if (sim_state.pos.z >= -3. && sim_state.airspeed > 0.) {
    double v2 = sim_state.airspeed * sim_state.airspeed;
    double vn2 = NOMINAL_AIRSPEED * NOMINAL_AIRSPEED;

    // roll dynamic
    double c_l = 4e-5 * sim_state.delta_roll;
    double phi_dot_dot = ROLL_RESPONSE_FACTOR * c_l * v2 / vn2 - sim_state.rates.phi;
    sim_state.rates.phi = sim_state.rates.phi + phi_dot_dot * fdm.curr_dt;
    BoundAbs(sim_state.rates.phi, ROLL_RATE_MAX_SETPOINT);
    sim_state.attitude.phi = sim_state.attitude.phi + sim_state.rates.phi * fdm.curr_dt;
    NormRadAngle(sim_state.attitude.phi);
    BoundAbs(sim_state.attitude.phi, ROLL_MAX_SETPOINT);

    // yaw dynamic
    sim_state.rates.psi = PPRZ_ISA_GRAVITY / sim_state.airspeed * tan(YAW_RESPONSE_FACTOR * sim_state.attitude.phi);
    sim_state.attitude.psi = sim_state.attitude.psi + sim_state.rates.psi * fdm.curr_dt;
    NormRadAngle(sim_state.attitude.psi);

    // Aerodynamic pitching moment coeff, proportional to elevator
    // No Thrust moment, so null (0) for steady flight
    double c_m = 5e-7 * sim_state.delta_pitch;
    double theta_dot_dot = PITCH_RESPONSE_FACTOR * c_m * v2 - sim_state.rates.theta;
    sim_state.rates.theta = sim_state.rates.theta + theta_dot_dot * fdm.curr_dt;
    sim_state.attitude.theta = sim_state.attitude.theta + sim_state.rates.theta * fdm.curr_dt;

    // Flight path angle
    double gamma = atan2(sim_state.speed.z, sim_state.airspeed);

    // Cz proportional to angle of attack
    double alpha = sim_state.attitude.theta - gamma;
    double c_z = 0.2 * alpha + PPRZ_ISA_GRAVITY / vn2;

    // Lift
    double lift = c_z * v2;
    double z_dot_dot = lift / WEIGHT * cos(sim_state.attitude.theta) * cos(sim_state.attitude.phi) - PPRZ_ISA_GRAVITY;
    sim_state.speed.z = sim_state.speed.z + z_dot_dot * fdm.curr_dt;
    sim_state.pos.z = sim_state.pos.z + sim_state.speed.z * fdm.curr_dt;

    // Constant Cx, drag to get expected cruise and maximum throttle
    double cruise_th = AUTO_THROTTLE_NOMINAL_CRUISE_THROTTLE;
    double drag = cruise_th + (v2 - vn2) * (1. - cruise_th) / (MAXIMUM_AIRSPEED*MAXIMUM_AIRSPEED - vn2);
    double airspeed_dot = MAXIMUM_POWER / sim_state.airspeed * (sim_state.delta_thrust - drag) / WEIGHT - PPRZ_ISA_GRAVITY * sin(gamma);
    sim_state.airspeed = sim_state.airspeed + airspeed_dot * fdm.curr_dt;
    sim_state.airspeed = Max(sim_state.airspeed, 10.); // avoid stall

    // Wind effect (FIXME apply to forces ?)
    sim_state.speed.x = sim_state.airspeed * sin(sim_state.attitude.psi) + sim_state.wind.x;
    sim_state.speed.y = sim_state.airspeed * cos(sim_state.attitude.psi) + sim_state.wind.y;
    sim_state.pos.x = sim_state.pos.x + sim_state.speed.x * fdm.curr_dt;
    sim_state.pos.y = sim_state.pos.y + sim_state.speed.y * fdm.curr_dt;
    sim_state.pos.z = sim_state.pos.z + sim_state.wind.z * fdm.curr_dt;
  }

  /****************************************************************************/
  // Coordenates transformations |
  // ----------------------------|

  /* in ECEF */
  ecef_of_enu_point_d(&fdm.ecef_pos, &ltpdef, &sim_state.pos);
  lla_of_ecef_d(&fdm.lla_pos, &fdm.ecef_pos);
  ecef_of_enu_vect_d(&fdm.ecef_ecef_vel, &ltpdef, &sim_state.speed);
  //ecef_of_enu_vect_d(&fdm.ecef_ecef_accel, &ltpdef, &rover_acc);

  /* in NED */
  ned_of_ecef_point_d(&fdm.ltpprz_pos, &ltpdef, &fdm.ecef_pos);
  ned_of_ecef_vect_d(&fdm.ltpprz_ecef_vel, &ltpdef, &fdm.ecef_ecef_vel);
  ned_of_ecef_vect_d(&fdm.ltpprz_ecef_accel, &ltpdef, &fdm.ecef_ecef_accel);
  fdm.hmsl = -fdm.ltpprz_pos.z + NAV_ALT0 / 1000.;

  /* Eulers */
  fdm.ltp_to_body_eulers = sim_state.attitude;
  // Exporting Eulers to AHRS (in quaternions)
  double_quat_of_eulers(&fdm.ltp_to_body_quat, &fdm.ltp_to_body_eulers);

  // Angular vel & acc
  fdm.body_ecef_rotvel.p = sim_state.rates.phi;
  fdm.body_ecef_rotvel.q = sim_state.rates.theta;
  fdm.body_ecef_rotvel.r = sim_state.rates.psi;
}


/**************************
 ** Generating LTP plane **
 **************************/

static void init_ltp(void)
{

  struct LlaCoor_d llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = RadOfDeg((double)NAV_LAT0 / 1e7);
  llh_nav0.lon = RadOfDeg((double)NAV_LON0 / 1e7);
  llh_nav0.alt = (double)(NAV_ALT0) / 1000.0;

  struct EcefCoor_d ecef_nav0;

  ecef_of_lla_d(&ecef_nav0, &llh_nav0);

  ltp_def_from_ecef_d(&ltpdef, &ecef_nav0);
  fdm.ecef_pos = ecef_nav0;

  // accel and mag data should not be used
  // AHRS and INS are bypassed
  fdm.ltp_g.x = 0.;
  fdm.ltp_g.y = 0.;
  fdm.ltp_g.z = 0.;

  fdm.ltp_h.x = 1.;
  fdm.ltp_h.y = 0.;
  fdm.ltp_h.z = 0.;

}


void nps_fdm_set_wind(double speed, double dir)
{
  sim_state.wind.x = speed * sin(dir);
  sim_state.wind.y = speed * cos(dir);
  sim_state.wind.z = 0.;
}

void nps_fdm_set_wind_ned(double wind_north, double wind_east, double wind_down)
{
  sim_state.wind.x = wind_east;
  sim_state.wind.y = wind_north;
  sim_state.wind.z = -wind_down;
}

void nps_fdm_set_turbulence(double wind_speed __attribute__((unused)),
                            int turbulence_severity __attribute__((unused)))
{
}

void nps_fdm_set_temperature(double temp __attribute__((unused)),
                             double h __attribute__((unused)))
{
}

