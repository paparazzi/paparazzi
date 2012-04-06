/*
 * Copyright (C) 2006  Pascal Brisset, Antoine Drouin
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
 * @file firmwares/fixedwing/guidance/guidance_v.h
 * Vertical control for fixed wing vehicles.
 *
 */

#ifndef FW_V_CTL_H
#define FW_V_CTL_H

#include <inttypes.h>
#include "paparazzi.h"

/* Vertical mode */
#define V_CTL_MODE_MANUAL        0
#define V_CTL_MODE_AUTO_THROTTLE 1
#define V_CTL_MODE_AUTO_CLIMB    2
#define V_CTL_MODE_AUTO_ALT      3
#define V_CTL_MODE_NB            4
extern uint8_t v_ctl_mode;

/* outer loop */
extern float v_ctl_altitude_error;    ///< in meters, (setpoint - alt) -> positive = too low
extern float v_ctl_altitude_setpoint; ///< in meters above MSL
extern float v_ctl_altitude_pre_climb;
extern float v_ctl_altitude_pgain;
extern float v_ctl_altitude_pre_climb_correction;
extern float v_ctl_altitude_max_climb;

/* inner loop */
extern float v_ctl_climb_setpoint;
extern uint8_t v_ctl_climb_mode;
#define V_CTL_CLIMB_MODE_AUTO_THROTTLE 0
#define V_CTL_CLIMB_MODE_AUTO_PITCH    1

extern uint8_t v_ctl_auto_throttle_submode;
#define V_CTL_AUTO_THROTTLE_STANDARD  0
#define V_CTL_AUTO_THROTTLE_AGRESSIVE 1
#define V_CTL_AUTO_THROTTLE_BLENDED   2

/* "auto throttle" inner loop parameters */
extern float v_ctl_auto_throttle_nominal_cruise_throttle;
extern float v_ctl_auto_throttle_min_cruise_throttle;
extern float v_ctl_auto_throttle_max_cruise_throttle;
extern float v_ctl_auto_throttle_cruise_throttle;
extern float v_ctl_auto_throttle_climb_throttle_increment;
extern float v_ctl_auto_throttle_pgain;
extern float v_ctl_auto_throttle_igain;
extern float v_ctl_auto_throttle_dgain;
extern float v_ctl_auto_throttle_sum_err;
extern float v_ctl_auto_throttle_pitch_of_vz_pgain;
extern float v_ctl_auto_throttle_pitch_of_vz_dgain;

/* agressive tuning */
#ifdef TUNE_AGRESSIVE_CLIMB
extern float agr_climb_throttle;
extern float agr_climb_pitch;
extern float agr_climb_nav_ratio;
extern float agr_descent_throttle;
extern float agr_descent_pitch;
extern float agr_descent_nav_ratio;
#endif

/* "auto pitch" inner loop parameters */
extern float v_ctl_auto_pitch_pgain;
extern float v_ctl_auto_pitch_igain;
extern float v_ctl_auto_pitch_sum_err;

extern pprz_t v_ctl_throttle_setpoint;
extern pprz_t v_ctl_throttle_slewed;

extern void v_ctl_init( void );
extern void v_ctl_altitude_loop( void );
extern void v_ctl_climb_loop ( void );

#if USE_AIRSPEED
/* "airspeed" inner loop parameters */
extern float v_ctl_auto_airspeed_setpoint;
extern float v_ctl_auto_airspeed_controlled;
extern float v_ctl_auto_airspeed_pgain;
extern float v_ctl_auto_airspeed_igain;
extern float v_ctl_auto_airspeed_sum_err;
extern float v_ctl_auto_groundspeed_setpoint;
extern float v_ctl_auto_groundspeed_pgain;
extern float v_ctl_auto_groundspeed_igain;
extern float v_ctl_auto_groundspeed_sum_err;
#endif

/** Computes throttle_slewed from throttle_setpoint */
extern void v_ctl_throttle_slew( void );

#define guidance_v_SetCruiseThrottle(_v) { \
  v_ctl_auto_throttle_cruise_throttle = (_v ? _v : v_ctl_auto_throttle_nominal_cruise_throttle); \
  Bound(v_ctl_auto_throttle_cruise_throttle, v_ctl_auto_throttle_min_cruise_throttle, v_ctl_auto_throttle_max_cruise_throttle); \
}

#define guidance_v_SetAutoThrottleIgain(_v) {	\
    v_ctl_auto_throttle_igain = _v;		\
    v_ctl_auto_throttle_sum_err = 0;		\
  }

#endif /* FW_V_CTL_H */
