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
 *
 */

#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include "std.h"

#include "led.h"

#include "airframe.h"
#include "booz2_ins.h"

#define BOOZ2_AP_MODE_FAILSAFE          0
#define BOOZ2_AP_MODE_KILL              1
#define BOOZ2_AP_MODE_RATE_DIRECT       2
#define BOOZ2_AP_MODE_ATTITUDE_DIRECT   3
#define BOOZ2_AP_MODE_RATE_RC_CLIMB     4
#define BOOZ2_AP_MODE_ATTITUDE_RC_CLIMB 5
#define BOOZ2_AP_MODE_ATTITUDE_CLIMB    6
#define BOOZ2_AP_MODE_RATE_Z_HOLD       7
#define BOOZ2_AP_MODE_ATTITUDE_Z_HOLD   8
#define BOOZ2_AP_MODE_HOVER_DIRECT      9
#define BOOZ2_AP_MODE_HOVER_CLIMB       10
#define BOOZ2_AP_MODE_HOVER_Z_HOLD      11
#define BOOZ2_AP_MODE_NAV               12


extern uint8_t autopilot_mode;
extern uint8_t autopilot_mode_auto2;
extern bool_t  autopilot_motors_on;
extern bool_t  autopilot_in_flight;
extern bool_t kill_throttle;
extern bool_t autopilot_rc;

extern bool_t autopilot_power_switch;

extern void autopilot_init(void);
extern void autopilot_periodic(void);
extern void autopilot_on_rc_frame(void);
extern void autopilot_set_mode(uint8_t new_autopilot_mode);

extern bool_t autopilot_detect_ground;
extern bool_t autopilot_detect_ground_once;

extern uint16_t autopilot_flight_time;

#ifndef BOOZ2_MODE_MANUAL
#define BOOZ2_MODE_MANUAL BOOZ2_AP_MODE_RATE_DIRECT
#endif
#ifndef BOOZ2_MODE_AUTO1
#define BOOZ2_MODE_AUTO1 BOOZ2_AP_MODE_ATTITUDE_DIRECT
#endif
#ifndef BOOZ2_MODE_AUTO2
#define BOOZ2_MODE_AUTO2 BOOZ2_AP_MODE_ATTITUDE_Z_HOLD
#endif


#define TRESHOLD_1_PPRZ (MIN_PPRZ / 2)
#define TRESHOLD_2_PPRZ (MAX_PPRZ / 2)

#define BOOZ_AP_MODE_OF_PPRZ(_rc, _booz_mode) {				\
    if      (_rc > TRESHOLD_2_PPRZ)					\
      _booz_mode = autopilot_mode_auto2;				\
    else if (_rc > TRESHOLD_1_PPRZ)					\
      _booz_mode = BOOZ2_MODE_AUTO1;					\
    else								\
      _booz_mode = BOOZ2_MODE_MANUAL;					\
  }

#define autopilot_KillThrottle(_v) {	                        \
    kill_throttle = _v;							\
    if (kill_throttle) autopilot_motors_on = FALSE;				\
    else autopilot_motors_on = TRUE; \
  }

#define autopilot_SetPowerSwitch(_v) { \
  autopilot_power_switch = _v; \
  if (_v) { LED_OFF(POWER_SWITCH_LED); } \
  else { LED_ON(POWER_SWITCH_LED); } \
}

#ifndef TRESHOLD_GROUND_DETECT
#define TRESHOLD_GROUND_DETECT ACCEL_BFP_OF_REAL(15.)
#endif
static inline void BoozDetectGroundEvent(void) {
  if (autopilot_mode == BOOZ2_AP_MODE_FAILSAFE || autopilot_detect_ground_once) {
    if (booz_ins_ltp_accel.z < -TRESHOLD_GROUND_DETECT ||
        booz_ins_ltp_accel.z > TRESHOLD_GROUND_DETECT) {
      autopilot_detect_ground = TRUE;
      autopilot_detect_ground_once = FALSE;
    }
  }
}

#endif /* AUTOPILOT_H */
