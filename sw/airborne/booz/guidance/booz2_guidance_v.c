/*
 * $Id$
 *
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

#define B2_GUIDANCE_V_C
#define B2_GUIDANCE_V_USE_REF
#include "booz2_guidance_v.h"


#include "booz_radio_control.h"
#include "booz_stabilization.h"
#include "booz_fms.h"
#include "booz2_navigation.h"

#include "booz2_ins.h"
#include "math/pprz_algebra_int.h"

#include "airframe.h"

uint8_t booz2_guidance_v_mode;
int32_t booz2_guidance_v_ff_cmd;
int32_t booz2_guidance_v_fb_cmd;
/* command output                                 */
int32_t booz2_guidance_v_delta_t;

/* direct throttle from radio control             */
/* range 0:200                                    */
int32_t booz2_guidance_v_rc_delta_t;
/* vertical speed setpoint from radio control     */
/* Q12.19 : accuracy 0.0000019, range +/-4096     */
int32_t booz2_guidance_v_rc_zd_sp;
/* altitude setpoint in meter (input)             */
/* Q23.8 : accuracy 0.0039, range 8388km          */
int32_t booz2_guidance_v_z_sp;
/* vertical speed setpoint in meter/s (input)     */
/* Q12.19 : accuracy 0.0000019, range +/-4096     */
int32_t booz2_guidance_v_zd_sp;
#define BOOZ2_GUIDANCE_V_ZD_SP_FRAC INT32_SPEED_FRAC

/* altitude reference in meter                    */
/* Q23.8 : accuracy 0.0039, range 8388km          */
int32_t booz2_guidance_v_z_ref;
/* vertical speed reference in meter/s            */
/* Q12.19 : accuracy 0.0000038, range 4096        */
int32_t booz2_guidance_v_zd_ref;
/* vertical acceleration reference in meter/s^2   */
/* Q21.10 : accuracy 0.0009766, range 2097152     */
int32_t booz2_guidance_v_zdd_ref;

int32_t booz2_guidance_v_kp;
int32_t booz2_guidance_v_kd;
int32_t booz2_guidance_v_ki;

int32_t booz2_guidance_v_z_sum_err;


#define Booz2GuidanceVSetRef(_pos, _speed, _accel) { \
    b2_gv_set_ref(_pos, _speed, _accel);	     \
    booz2_guidance_v_z_ref = _pos;		     \
    booz2_guidance_v_zd_ref = _speed;		     \
    booz2_guidance_v_zdd_ref = _accel;		     \
  }


static inline void run_hover_loop(bool_t in_flight);


void booz2_guidance_v_init(void) {

  booz2_guidance_v_mode = BOOZ2_GUIDANCE_V_MODE_KILL;

  booz2_guidance_v_kp = BOOZ2_GUIDANCE_V_HOVER_KP;
  booz2_guidance_v_kd = BOOZ2_GUIDANCE_V_HOVER_KD;
  booz2_guidance_v_ki = BOOZ2_GUIDANCE_V_HOVER_KI;

  booz2_guidance_v_z_sum_err = 0;

  b2_gv_adapt_init();
}


void booz2_guidance_v_read_rc(void) {

  // used in RC_DIRECT directly and as saturation in CLIMB and HOVER
  booz2_guidance_v_rc_delta_t = (int32_t)radio_control.values[RADIO_CONTROL_THROTTLE] * 200 / MAX_PPRZ;
  // used in RC_CLIMB
  booz2_guidance_v_rc_zd_sp   = ((MAX_PPRZ/2) - (int32_t)radio_control.values[RADIO_CONTROL_THROTTLE]) *
                                BOOZ2_GUIDANCE_V_RC_CLIMB_COEF;
  DeadBand(booz2_guidance_v_rc_zd_sp, BOOZ2_GUIDANCE_V_RC_CLIMB_DEAD_BAND);

}

void booz2_guidance_v_mode_changed(uint8_t new_mode) {

  if (new_mode == booz2_guidance_v_mode)
    return;

  //  switch ( booz2_guidance_v_mode ) {
  //
  //  }

  switch (new_mode) {

  case BOOZ2_GUIDANCE_V_MODE_RC_CLIMB:
  case BOOZ2_GUIDANCE_V_MODE_CLIMB:
  case BOOZ2_GUIDANCE_V_MODE_HOVER:
  case BOOZ2_GUIDANCE_V_MODE_NAV:
    booz2_guidance_v_z_sum_err = 0;
    Booz2GuidanceVSetRef(booz_ins_ltp_pos.z, booz_ins_ltp_speed.z, 0);
    break;

  }


  booz2_guidance_v_mode = new_mode;

}

void booz2_guidance_v_notify_in_flight( bool_t in_flight) {
  if (in_flight)
    b2_gv_adapt_init();
}


void booz2_guidance_v_run(bool_t in_flight) {

  // FIXME... SATURATIONS NOT TAKEN INTO ACCOUNT
  // AKA SUPERVISION and co
  if (in_flight) {
    // we should use something after the supervision!!! fuck!!!
    int32_t cmd_hack = Chop(booz_stabilization_cmd[COMMAND_THRUST], 1, 200);
    b2_gv_adapt_run(booz_ins_ltp_accel.z, cmd_hack);
  }
  else {
    // reset vertical filter until takeoff
    //booz_ins_vf_realign = TRUE;
  }

  switch (booz2_guidance_v_mode) {

  case BOOZ2_GUIDANCE_V_MODE_RC_DIRECT:
    booz2_guidance_v_z_sp = booz_ins_ltp_pos.z;  // not sure why we do that
    Booz2GuidanceVSetRef(booz_ins_ltp_pos.z, 0, 0); // or that - mode enter should take care of it ?
    booz_stabilization_cmd[COMMAND_THRUST] = booz2_guidance_v_rc_delta_t;
    break;

  case BOOZ2_GUIDANCE_V_MODE_RC_CLIMB:
    booz2_guidance_v_zd_sp = booz2_guidance_v_rc_zd_sp;
    b2_gv_update_ref_from_zd_sp(booz2_guidance_v_zd_sp);
    run_hover_loop(in_flight);
    booz_stabilization_cmd[COMMAND_THRUST] = booz2_guidance_v_delta_t;
    break;

  case BOOZ2_GUIDANCE_V_MODE_CLIMB:
#ifdef USE_FMS
    if (fms.enabled && fms.input.v_mode == BOOZ2_GUIDANCE_V_MODE_CLIMB)
      booz2_guidance_v_zd_sp = fms.input.v_sp.climb;
#endif
    b2_gv_update_ref_from_zd_sp(booz2_guidance_v_zd_sp);
    run_hover_loop(in_flight);
    // saturate max authority with RC stick
    booz_stabilization_cmd[COMMAND_THRUST] = Min( booz2_guidance_v_rc_delta_t, booz2_guidance_v_delta_t);
    break;

  case BOOZ2_GUIDANCE_V_MODE_HOVER:
#ifdef USE_FMS
    if (fms.enabled && fms.input.v_mode == BOOZ2_GUIDANCE_V_MODE_HOVER)
      booz2_guidance_v_z_sp = fms.input.v_sp.height;
#endif
    b2_gv_update_ref_from_z_sp(booz2_guidance_v_z_sp);
    run_hover_loop(in_flight);
    // saturate max authority with RC stick
    booz_stabilization_cmd[COMMAND_THRUST] = Min( booz2_guidance_v_rc_delta_t, booz2_guidance_v_delta_t);
    break;

  case BOOZ2_GUIDANCE_V_MODE_NAV:
    {
      if (vertical_mode == VERTICAL_MODE_ALT) {
        booz2_guidance_v_z_sp = -nav_flight_altitude;
        b2_gv_update_ref_from_z_sp(booz2_guidance_v_z_sp);
        run_hover_loop(in_flight);
      }
      else if (vertical_mode == VERTICAL_MODE_CLIMB) {
        booz2_guidance_v_zd_sp = -nav_climb;
        b2_gv_update_ref_from_zd_sp(booz2_guidance_v_zd_sp);
        nav_flight_altitude = -booz2_guidance_v_z_sp;
        run_hover_loop(in_flight);
      }
      else if (vertical_mode == VERTICAL_MODE_MANUAL) {
        booz2_guidance_v_z_sp = -nav_flight_altitude; // For display only
        booz2_guidance_v_delta_t = nav_throttle;
      }
      /* use rc limitation if available */
      if (radio_control.status == RADIO_CONTROL_OK)
        booz_stabilization_cmd[COMMAND_THRUST] = Min( booz2_guidance_v_rc_delta_t, booz2_guidance_v_delta_t);
      else
        booz_stabilization_cmd[COMMAND_THRUST] = booz2_guidance_v_delta_t;
      break;
    }
  }
}


#define FF_CMD_FRAC 18

static inline void run_hover_loop(bool_t in_flight) {

  /* convert our reference to generic representation */
  int64_t tmp  = b2_gv_z_ref>>(B2_GV_Z_REF_FRAC - INT32_POS_FRAC);
  booz2_guidance_v_z_ref = (int32_t)tmp;
  booz2_guidance_v_zd_ref = b2_gv_zd_ref<<(INT32_SPEED_FRAC - B2_GV_ZD_REF_FRAC);
  booz2_guidance_v_zdd_ref = b2_gv_zdd_ref<<(INT32_ACCEL_FRAC - B2_GV_ZDD_REF_FRAC);
  /* compute the error to our reference */
  int32_t err_z  =  booz_ins_ltp_pos.z - booz2_guidance_v_z_ref;
  Bound(err_z, BOOZ2_GUIDANCE_V_MIN_ERR_Z, BOOZ2_GUIDANCE_V_MAX_ERR_Z);
  int32_t err_zd =  booz_ins_ltp_speed.z - booz2_guidance_v_zd_ref;
  Bound(err_zd, BOOZ2_GUIDANCE_V_MIN_ERR_ZD, BOOZ2_GUIDANCE_V_MAX_ERR_ZD);

  if (in_flight) {
    booz2_guidance_v_z_sum_err += err_z;
    Bound(booz2_guidance_v_z_sum_err, -BOOZ2_GUIDANCE_V_MAX_SUM_ERR, BOOZ2_GUIDANCE_V_MAX_SUM_ERR);
  }
  else
    booz2_guidance_v_z_sum_err = 0;

  /* our nominal command : (g + zdd)*m   */
#ifdef BOOZ2_GUIDANCE_V_INV_M
  const int32_t inv_m = BFP_OF_REAL(BOOZ2_GUIDANCE_V_INV_M, B2_GV_ADAPT_X_FRAC);
#else
  const int32_t inv_m =  b2_gv_adapt_X>>(B2_GV_ADAPT_X_FRAC - FF_CMD_FRAC);
#endif
  const int32_t g_m_zdd = (int32_t)BFP_OF_REAL(9.81, FF_CMD_FRAC) -
                          (booz2_guidance_v_zdd_ref<<(FF_CMD_FRAC - INT32_ACCEL_FRAC));
#if 0
  if (g_m_zdd > 0)
    booz2_guidance_v_ff_cmd = ( g_m_zdd + (inv_m>>1)) / inv_m;
  else
    booz2_guidance_v_ff_cmd = ( g_m_zdd - (inv_m>>1)) / inv_m;
#else
  booz2_guidance_v_ff_cmd = g_m_zdd / inv_m;
#endif
  //  booz2_guidance_v_ff_cmd = BOOZ2_GUIDANCE_V_HOVER_POWER;

  /* our error command                   */
  booz2_guidance_v_fb_cmd = ((-booz2_guidance_v_kp * err_z)  >> 12) +
                            ((-booz2_guidance_v_kd * err_zd) >> 21) +
                            ((-booz2_guidance_v_ki * booz2_guidance_v_z_sum_err) >> 21);

  booz2_guidance_v_delta_t = booz2_guidance_v_ff_cmd + booz2_guidance_v_fb_cmd;
  // booz2_guidance_v_delta_t = booz2_guidance_v_fb_cmd;


}

