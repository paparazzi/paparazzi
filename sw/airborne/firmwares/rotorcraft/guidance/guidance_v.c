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

#define GUIDANCE_V_C
#define GUIDANCE_V_USE_REF
#include "firmwares/rotorcraft/guidance/guidance_v.h"


#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "subsystems/ahrs.h"
// #include "booz_fms.h" FIXME
#include "firmwares/rotorcraft/navigation.h"

#include "subsystems/ins.h"
#include "math/pprz_algebra_int.h"

#include "generated/airframe.h"

/* In case Asctec controllers are used without supervision */
#ifndef SUPERVISION_MIN_MOTOR
#define SUPERVISION_MIN_MOTOR 1
#endif
#ifndef SUPERVISION_MAX_MOTOR
#define SUPERVISION_MAX_MOTOR 200
#endif

uint8_t guidance_v_mode;
int32_t guidance_v_ff_cmd;
int32_t guidance_v_fb_cmd;
/* command output                                 */
int32_t guidance_v_delta_t;

/* direct throttle from radio control             */
/* range 0:200                                    */
int32_t guidance_v_rc_delta_t;
/* vertical speed setpoint from radio control     */
/* Q12.19 : accuracy 0.0000019, range +/-4096     */
int32_t guidance_v_rc_zd_sp;
/* altitude setpoint in meter (input)             */
/* Q23.8 : accuracy 0.0039, range 8388km          */
int32_t guidance_v_z_sp;
/* vertical speed setpoint in meter/s (input)     */
/* Q12.19 : accuracy 0.0000019, range +/-4096     */
int32_t guidance_v_zd_sp;
#define GUIDANCE_V_ZD_SP_FRAC INT32_SPEED_FRAC

/* altitude reference in meter                    */
/* Q23.8 : accuracy 0.0039, range 8388km          */
int32_t guidance_v_z_ref;
/* vertical speed reference in meter/s            */
/* Q12.19 : accuracy 0.0000038, range 4096        */
int32_t guidance_v_zd_ref;
/* vertical acceleration reference in meter/s^2   */
/* Q21.10 : accuracy 0.0009766, range 2097152     */
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


__attribute__ ((always_inline)) static inline void run_hover_loop(bool_t in_flight);


void guidance_v_init(void) {

  guidance_v_mode = GUIDANCE_V_MODE_KILL;

  guidance_v_kp = GUIDANCE_V_HOVER_KP;
  guidance_v_kd = GUIDANCE_V_HOVER_KD;
  guidance_v_ki = GUIDANCE_V_HOVER_KI;

  guidance_v_z_sum_err = 0;

  gv_adapt_init();
}


void guidance_v_read_rc(void) {

  // used in RC_DIRECT directly and as saturation in CLIMB and HOVER
#ifndef USE_HELI
  guidance_v_rc_delta_t = (int32_t)radio_control.values[RADIO_THROTTLE] * 200 / MAX_PPRZ;
#else
  guidance_v_rc_delta_t = (int32_t)radio_control.values[RADIO_THROTTLE] * 4 / 5;
#endif
  // used in RC_CLIMB
  guidance_v_rc_zd_sp   = ((MAX_PPRZ/2) - (int32_t)radio_control.values[RADIO_THROTTLE]) *
                                GUIDANCE_V_RC_CLIMB_COEF;
  DeadBand(guidance_v_rc_zd_sp, GUIDANCE_V_RC_CLIMB_DEAD_BAND);

}

void guidance_v_mode_changed(uint8_t new_mode) {

  if (new_mode == guidance_v_mode)
    return;

  //  switch ( guidance_v_mode ) {
  //
  //  }

  switch (new_mode) {

  case GUIDANCE_V_MODE_RC_CLIMB:
  case GUIDANCE_V_MODE_CLIMB:
  case GUIDANCE_V_MODE_HOVER:
  case GUIDANCE_V_MODE_NAV:
    guidance_v_z_sum_err = 0;
    GuidanceVSetRef(ins_ltp_pos.z, ins_ltp_speed.z, 0);
    break;
  default:
    break;
  }


  guidance_v_mode = new_mode;

}

void guidance_v_notify_in_flight( bool_t in_flight) {
  if (in_flight)
    gv_adapt_init();
}


void guidance_v_run(bool_t in_flight) {

  // FIXME... SATURATIONS NOT TAKEN INTO ACCOUNT
  // AKA SUPERVISION and co
  if (in_flight) {
    // we should use something after the supervision!!! fuck!!!
    int32_t cmd_hack = Chop(stabilization_cmd[COMMAND_THRUST], SUPERVISION_MIN_MOTOR, SUPERVISION_MAX_MOTOR);
    gv_adapt_run(ins_ltp_accel.z, cmd_hack);
    //gv_adapt_run(ins_ltp_accel.z, cmd_hack, guidance_v_zd_ref);
  }
  else {
    // reset vertical filter until takeoff
    //ins_vf_realign = TRUE;
  }

  switch (guidance_v_mode) {

  case GUIDANCE_V_MODE_RC_DIRECT:
    guidance_v_z_sp = ins_ltp_pos.z;  // not sure why we do that
    GuidanceVSetRef(ins_ltp_pos.z, 0, 0); // or that - mode enter should take care of it ?
    stabilization_cmd[COMMAND_THRUST] = guidance_v_rc_delta_t;
    break;

  case GUIDANCE_V_MODE_RC_CLIMB:
    guidance_v_zd_sp = guidance_v_rc_zd_sp;
    gv_update_ref_from_zd_sp(guidance_v_zd_sp);
    run_hover_loop(in_flight);
    stabilization_cmd[COMMAND_THRUST] = guidance_v_delta_t;
    break;

  case GUIDANCE_V_MODE_CLIMB:
#ifdef USE_FMS
    if (fms.enabled && fms.input.v_mode == GUIDANCE_V_MODE_CLIMB)
      guidance_v_zd_sp = fms.input.v_sp.climb;
#endif
    gv_update_ref_from_zd_sp(guidance_v_zd_sp);
    run_hover_loop(in_flight);
    // saturate max authority with RC stick
    stabilization_cmd[COMMAND_THRUST] = Min( guidance_v_rc_delta_t, guidance_v_delta_t);
    break;

  case GUIDANCE_V_MODE_HOVER:
#ifdef USE_FMS
    if (fms.enabled && fms.input.v_mode == GUIDANCE_V_MODE_HOVER)
      guidance_v_z_sp = fms.input.v_sp.height;
#endif
    gv_update_ref_from_z_sp(guidance_v_z_sp);
    run_hover_loop(in_flight);
    // saturate max authority with RC stick
    stabilization_cmd[COMMAND_THRUST] = Min( guidance_v_rc_delta_t, guidance_v_delta_t);
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
      /* use rc limitation if available */
      if (radio_control.status == RC_OK)
        stabilization_cmd[COMMAND_THRUST] = Min( guidance_v_rc_delta_t, guidance_v_delta_t);
      else
        stabilization_cmd[COMMAND_THRUST] = guidance_v_delta_t;
      break;
    }
  default:
    break;
  }
}


#define FF_CMD_FRAC 18

#define MAX_BANK_COEF (BFP_OF_REAL(RadOfDeg(30.),INT32_TRIG_FRAC))

__attribute__ ((always_inline)) static inline void run_hover_loop(bool_t in_flight) {

  /* convert our reference to generic representation */
  int64_t tmp  = gv_z_ref>>(GV_Z_REF_FRAC - INT32_POS_FRAC);
  guidance_v_z_ref = (int32_t)tmp;
  guidance_v_zd_ref = gv_zd_ref<<(INT32_SPEED_FRAC - GV_ZD_REF_FRAC);
  guidance_v_zdd_ref = gv_zdd_ref<<(INT32_ACCEL_FRAC - GV_ZDD_REF_FRAC);
  /* compute the error to our reference */
  int32_t err_z  =  ins_ltp_pos.z - guidance_v_z_ref;
  Bound(err_z, GUIDANCE_V_MIN_ERR_Z, GUIDANCE_V_MAX_ERR_Z);
  int32_t err_zd =  ins_ltp_speed.z - guidance_v_zd_ref;
  Bound(err_zd, GUIDANCE_V_MIN_ERR_ZD, GUIDANCE_V_MAX_ERR_ZD);

  if (in_flight) {
    guidance_v_z_sum_err += err_z;
    Bound(guidance_v_z_sum_err, -GUIDANCE_V_MAX_SUM_ERR, GUIDANCE_V_MAX_SUM_ERR);
  }
  else
    guidance_v_z_sum_err = 0;

  /* our nominal command : (g + zdd)*m   */
#ifdef GUIDANCE_V_INV_M
  const int32_t inv_m = BFP_OF_REAL(GUIDANCE_V_INV_M, GV_ADAPT_X_FRAC);
#else
  const int32_t inv_m =  gv_adapt_X>>(GV_ADAPT_X_FRAC - FF_CMD_FRAC);
#endif
  const int32_t g_m_zdd = (int32_t)BFP_OF_REAL(9.81, FF_CMD_FRAC) -
                          (guidance_v_zdd_ref<<(FF_CMD_FRAC - INT32_ACCEL_FRAC));
#if 0
  if (g_m_zdd > 0)
    guidance_v_ff_cmd = ( g_m_zdd + (inv_m>>1)) / inv_m;
  else
    guidance_v_ff_cmd = ( g_m_zdd - (inv_m>>1)) / inv_m;
#else
  guidance_v_ff_cmd = g_m_zdd / inv_m;
  int32_t cphi,ctheta,cphitheta;
  PPRZ_ITRIG_COS(cphi, ahrs.ltp_to_body_euler.phi);
  PPRZ_ITRIG_COS(ctheta, ahrs.ltp_to_body_euler.theta);
  cphitheta = (cphi * ctheta) >> INT32_TRIG_FRAC;
  if (cphitheta < MAX_BANK_COEF) cphitheta = MAX_BANK_COEF;
  guidance_v_ff_cmd = (guidance_v_ff_cmd << INT32_TRIG_FRAC) / cphitheta;
#endif

  /* our error command                   */
  guidance_v_fb_cmd = ((-guidance_v_kp * err_z)  >> 12) +
                            ((-guidance_v_kd * err_zd) >> 21) +
                            ((-guidance_v_ki * guidance_v_z_sum_err) >> 21);

  guidance_v_delta_t = guidance_v_ff_cmd + guidance_v_fb_cmd;
  // guidance_v_delta_t = guidance_v_fb_cmd;


}
