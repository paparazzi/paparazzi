/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

/** @file firmwares/rotorcraft/guidance/guidance_v.c
 *  Vertical guidance for rotorcrafts.
 *
 */

#include "generated/airframe.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"

#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/navigation.h"

#include "state.h"

#include "math/pprz_algebra_int.h"


/* error if some gains are negative */
#if (GUIDANCE_V_HOVER_KP < 0) ||                   \
  (GUIDANCE_V_HOVER_KD < 0)   ||                   \
  (GUIDANCE_V_HOVER_KI < 0)
#error "ALL control gains must be positive!!!"
#endif


/* If only GUIDANCE_V_NOMINAL_HOVER_THROTTLE is defined,
 * disable the adaptive throttle estimation by default.
 * Otherwise enable adaptive estimation by default.
 */
#ifdef GUIDANCE_V_NOMINAL_HOVER_THROTTLE
#  ifndef GUIDANCE_V_ADAPT_THROTTLE_ENABLED
#    define GUIDANCE_V_ADAPT_THROTTLE_ENABLED FALSE
#  endif
#else
#  define GUIDANCE_V_NOMINAL_HOVER_THROTTLE 0.4
#  ifndef GUIDANCE_V_ADAPT_THROTTLE_ENABLED
#    define GUIDANCE_V_ADAPT_THROTTLE_ENABLED TRUE
#  endif
#endif
PRINT_CONFIG_VAR(GUIDANCE_V_NOMINAL_HOVER_THROTTLE)
PRINT_CONFIG_VAR(GUIDANCE_V_ADAPT_THROTTLE_ENABLED)


uint8_t guidance_v_mode;
int32_t guidance_v_ff_cmd;
int32_t guidance_v_fb_cmd;
int32_t guidance_v_delta_t;

float guidance_v_nominal_throttle;
bool_t guidance_v_adapt_throttle_enabled;


/** Direct throttle from radio control.
 *  range 0:#MAX_PPRZ
 */
int32_t guidance_v_rc_delta_t;

/** Vertical speed setpoint from radio control.
 *  fixed point representation: Q12.19
 *  accuracy 0.0000019, range +/-4096
 */
int32_t guidance_v_rc_zd_sp;

int32_t guidance_v_z_sp;
int32_t guidance_v_zd_sp;
int32_t guidance_v_z_ref;
int32_t guidance_v_zd_ref;
int32_t guidance_v_zdd_ref;

int32_t guidance_v_kp;
int32_t guidance_v_kd;
int32_t guidance_v_ki;

int32_t guidance_v_z_sum_err;


#define GuidanceVSetRef(_pos, _speed, _accel) { \
    gv_set_ref(_pos, _speed, _accel);	     \
    guidance_v_z_ref = _pos;		     \
    guidance_v_zd_ref = _speed;		     \
    guidance_v_zdd_ref = _accel;		     \
  }


void run_hover_loop(bool_t in_flight);


void guidance_v_init(void) {

  guidance_v_mode = GUIDANCE_V_MODE_KILL;

  guidance_v_kp = GUIDANCE_V_HOVER_KP;
  guidance_v_kd = GUIDANCE_V_HOVER_KD;
  guidance_v_ki = GUIDANCE_V_HOVER_KI;

  guidance_v_z_sum_err = 0;

  guidance_v_nominal_throttle = GUIDANCE_V_NOMINAL_HOVER_THROTTLE;
  guidance_v_adapt_throttle_enabled = GUIDANCE_V_ADAPT_THROTTLE_ENABLED;

  gv_adapt_init();
}


void guidance_v_read_rc(void) {

  /* used in RC_DIRECT directly and as saturation in CLIMB and HOVER */
  guidance_v_rc_delta_t = (int32_t)radio_control.values[RADIO_THROTTLE];

  /* used in RC_CLIMB */
  guidance_v_rc_zd_sp = ((MAX_PPRZ/2) - (int32_t)radio_control.values[RADIO_THROTTLE]) * GUIDANCE_V_RC_CLIMB_COEF;
  DeadBand(guidance_v_rc_zd_sp, GUIDANCE_V_RC_CLIMB_DEAD_BAND);

}

void guidance_v_mode_changed(uint8_t new_mode) {

  if (new_mode == guidance_v_mode)
    return;

  switch (new_mode) {
  case GUIDANCE_V_MODE_HOVER:
    guidance_v_z_sp = stateGetPositionNed_i()->z; // set current altitude as setpoint
    guidance_v_z_sum_err = 0;
    GuidanceVSetRef(stateGetPositionNed_i()->z, 0, 0);
    break;

  case GUIDANCE_V_MODE_RC_CLIMB:
  case GUIDANCE_V_MODE_CLIMB:
    guidance_v_zd_sp = 0;
  case GUIDANCE_V_MODE_NAV:
    guidance_v_z_sum_err = 0;
    GuidanceVSetRef(stateGetPositionNed_i()->z, stateGetSpeedNed_i()->z, 0);
    break;

  default:
    break;

  }

  guidance_v_mode = new_mode;

}

void guidance_v_notify_in_flight( bool_t in_flight) {
  if (in_flight) {
    gv_adapt_init();
  }
}


void guidance_v_run(bool_t in_flight) {

  // FIXME... SATURATIONS NOT TAKEN INTO ACCOUNT
  // AKA SUPERVISION and co
  if (in_flight) {
    gv_adapt_run(stateGetAccelNed_i()->z, stabilization_cmd[COMMAND_THRUST], guidance_v_zd_ref);
  }
  else {
    /* reset estimate while not in_flight */
    gv_adapt_init();
  }

  switch (guidance_v_mode) {

  case GUIDANCE_V_MODE_RC_DIRECT:
    guidance_v_z_sp = stateGetPositionNed_i()->z; // for display only
    stabilization_cmd[COMMAND_THRUST] = guidance_v_rc_delta_t;
    break;

  case GUIDANCE_V_MODE_RC_CLIMB:
    guidance_v_zd_sp = guidance_v_rc_zd_sp;
    gv_update_ref_from_zd_sp(guidance_v_zd_sp);
    run_hover_loop(in_flight);
    stabilization_cmd[COMMAND_THRUST] = guidance_v_delta_t;
    break;

  case GUIDANCE_V_MODE_CLIMB:
#if USE_FMS
    if (fms.enabled && fms.input.v_mode == GUIDANCE_V_MODE_CLIMB) {
      guidance_v_zd_sp = fms.input.v_sp.climb;
    }
#endif
    gv_update_ref_from_zd_sp(guidance_v_zd_sp);
    run_hover_loop(in_flight);
#if NO_RC_THRUST_LIMIT
    stabilization_cmd[COMMAND_THRUST] = guidance_v_delta_t;
#else
    // saturate max authority with RC stick
    stabilization_cmd[COMMAND_THRUST] = Min(guidance_v_rc_delta_t, guidance_v_delta_t);
#endif
    break;

  case GUIDANCE_V_MODE_HOVER:
#if USE_FMS
    if (fms.enabled && fms.input.v_mode == GUIDANCE_V_MODE_HOVER)
      guidance_v_z_sp = fms.input.v_sp.height;
#endif
    gv_update_ref_from_z_sp(guidance_v_z_sp);
    run_hover_loop(in_flight);
#if NO_RC_THRUST_LIMIT
    stabilization_cmd[COMMAND_THRUST] = guidance_v_delta_t;
#else
    // saturate max authority with RC stick
    stabilization_cmd[COMMAND_THRUST] = Min(guidance_v_rc_delta_t, guidance_v_delta_t);
#endif
    break;

  case GUIDANCE_V_MODE_NAV:
    {
      if (vertical_mode == VERTICAL_MODE_ALT) {
        guidance_v_z_sp = -nav_flight_altitude;
        gv_update_ref_from_z_sp(guidance_v_z_sp);
        run_hover_loop(in_flight);
      }
      else if (vertical_mode == VERTICAL_MODE_CLIMB) {
        guidance_v_zd_sp = -nav_climb;
        gv_update_ref_from_zd_sp(guidance_v_zd_sp);
        nav_flight_altitude = -guidance_v_z_sp;
        run_hover_loop(in_flight);
      }
      else if (vertical_mode == VERTICAL_MODE_MANUAL) {
        guidance_v_z_sp = -nav_flight_altitude; // For display only
        guidance_v_delta_t = nav_throttle;
      }
#if NO_RC_THRUST_LIMIT
      stabilization_cmd[COMMAND_THRUST] = guidance_v_delta_t;
#else
      /* use rc limitation if available */
      if (radio_control.status == RC_OK)
        stabilization_cmd[COMMAND_THRUST] = Min(guidance_v_rc_delta_t, guidance_v_delta_t);
      else
        stabilization_cmd[COMMAND_THRUST] = guidance_v_delta_t;
#endif
      break;
    }
  default:
    break;
  }
}


#define FF_CMD_FRAC 18

#define MAX_BANK_COEF (BFP_OF_REAL(RadOfDeg(30.),INT32_TRIG_FRAC))

void run_hover_loop(bool_t in_flight) {

  /* convert our reference to generic representation */
  int64_t tmp  = gv_z_ref>>(GV_Z_REF_FRAC - INT32_POS_FRAC);
  guidance_v_z_ref = (int32_t)tmp;
  guidance_v_zd_ref = gv_zd_ref<<(INT32_SPEED_FRAC - GV_ZD_REF_FRAC);
  guidance_v_zdd_ref = gv_zdd_ref<<(INT32_ACCEL_FRAC - GV_ZDD_REF_FRAC);
  /* compute the error to our reference */
  int32_t err_z  = guidance_v_z_ref - stateGetPositionNed_i()->z;
  Bound(err_z, GUIDANCE_V_MIN_ERR_Z, GUIDANCE_V_MAX_ERR_Z);
  int32_t err_zd = guidance_v_zd_ref - stateGetSpeedNed_i()->z;
  Bound(err_zd, GUIDANCE_V_MIN_ERR_ZD, GUIDANCE_V_MAX_ERR_ZD);

  if (in_flight) {
    guidance_v_z_sum_err += err_z;
    Bound(guidance_v_z_sum_err, -GUIDANCE_V_MAX_SUM_ERR, GUIDANCE_V_MAX_SUM_ERR);
  }
  else
    guidance_v_z_sum_err = 0;

  /* our nominal command : (g + zdd)*m   */
  int32_t inv_m;
  if (guidance_v_adapt_throttle_enabled) {
    inv_m =  gv_adapt_X >> (GV_ADAPT_X_FRAC - FF_CMD_FRAC);
  }
  else {
    /* use the fixed nominal throttle */
    inv_m = BFP_OF_REAL(9.81 / (guidance_v_nominal_throttle * MAX_PPRZ), FF_CMD_FRAC);
  }

  const int32_t g_m_zdd = (int32_t)BFP_OF_REAL(9.81, FF_CMD_FRAC) -
                          (guidance_v_zdd_ref << (FF_CMD_FRAC - INT32_ACCEL_FRAC));

  guidance_v_ff_cmd = g_m_zdd / inv_m;
  int32_t cphi,ctheta,cphitheta;
  struct Int32Eulers* att_euler = stateGetNedToBodyEulers_i();
  PPRZ_ITRIG_COS(cphi, att_euler->phi);
  PPRZ_ITRIG_COS(ctheta, att_euler->theta);
  cphitheta = (cphi * ctheta) >> INT32_TRIG_FRAC;
  if (cphitheta < MAX_BANK_COEF) cphitheta = MAX_BANK_COEF;
  /* feed forward command */
  guidance_v_ff_cmd = (guidance_v_ff_cmd << INT32_TRIG_FRAC) / cphitheta;

  /* bound the nominal command to 0.9*MAX_PPRZ */
  Bound(guidance_v_ff_cmd, 0, 8640);


  /* our error feed back command                   */
  /* z-axis pointing down -> positive error means we need less thrust */
  guidance_v_fb_cmd = ((-guidance_v_kp * err_z)  >> 7) +
                      ((-guidance_v_kd * err_zd) >> 16) +
                      ((-guidance_v_ki * guidance_v_z_sum_err) >> 16);

  guidance_v_delta_t = guidance_v_ff_cmd + guidance_v_fb_cmd;

  /* bound the result */
  Bound(guidance_v_delta_t, 0, MAX_PPRZ);

}
