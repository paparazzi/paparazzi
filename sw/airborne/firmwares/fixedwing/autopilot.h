/*
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
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
 * @file firmwares/fixedwing/autopilot.h
 *
 * Fixedwing autopilot modes.
 *
 */

#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include <inttypes.h>
#include "std.h"
#include "paparazzi.h"
#include "mcu_periph/sys_time.h"
#include "generated/airframe.h"


/** Autopilot inititalization.
 */
extern void autopilot_init(void);

/** Threshold for RC mode detection.
 */
#define THRESHOLD_MANUAL_PPRZ (MIN_PPRZ / 2)
#define THRESHOLD1 THRESHOLD_MANUAL_PPRZ
#define THRESHOLD2 (MAX_PPRZ/2)

/** AP modes.
 */
#define  PPRZ_MODE_MANUAL 0
#define  PPRZ_MODE_AUTO1 1
#define  PPRZ_MODE_AUTO2 2
#define  PPRZ_MODE_HOME	3
#define  PPRZ_MODE_GPS_OUT_OF_ORDER 4
#define  PPRZ_MODE_NB 5

#define PPRZ_MODE_OF_PULSE(pprz) \
  (pprz > THRESHOLD2 ? PPRZ_MODE_AUTO2 : \
        (pprz > THRESHOLD1 ? PPRZ_MODE_AUTO1 : PPRZ_MODE_MANUAL))

extern uint8_t pprz_mode;
extern bool_t kill_throttle;

/** flight time in seconds. */
extern uint16_t autopilot_flight_time;

#define autopilot_ResetFlightTimeAndLaunch(_) { \
  autopilot_flight_time = 0; launch = FALSE; \
}


// FIXME, move to control
#define LATERAL_MODE_MANUAL    0
#define LATERAL_MODE_ROLL_RATE 1
#define LATERAL_MODE_ROLL      2
#define LATERAL_MODE_COURSE    3
#define LATERAL_MODE_NB        4
extern uint8_t lateral_mode;

#define STICK_PUSHED(pprz) (pprz < THRESHOLD1 || pprz > THRESHOLD2)
#define FLOAT_OF_PPRZ(pprz, center, travel) ((float)pprz / (float)MAX_PPRZ * travel + center)

#define THROTTLE_THRESHOLD_TAKEOFF (pprz_t)(MAX_PPRZ * 0.9)

/** Supply voltage in deciVolt.
 * This the ap copy of the measurement from fbw
 */
extern uint16_t vsupply;

/** Fuel consumption (mAh)
 * TODO: move to electrical subsystem
 */
extern float energy;

extern bool_t launch;

extern bool_t gps_lost;

/** Assignment, returning _old_value != _value
 * Using GCC expression statements */
#define ModeUpdate(_mode, _value) ({ \
  uint8_t new_mode = _value; \
  (_mode != new_mode ? _mode = new_mode, TRUE : FALSE); \
})


/** Power switch control.
 */
extern bool_t power_switch;

#ifdef POWER_SWITCH_LED
#define autopilot_SetPowerSwitch(_x) { \
  power_switch = _x; \
  if (_x) LED_ON(POWER_SWITCH_LED) else LED_OFF(POWER_SWITCH_LED); \
}
#else // POWER_SWITCH_LED
#define autopilot_SetPowerSwitch(_x) { power_switch = _x; }
#endif // POWER_SWITCH_LED


/* CONTROL_RATE will be removed in the next release
 * please use CONTROL_FREQUENCY instead
 */
#ifndef CONTROL_FREQUENCY
#ifdef  CONTROL_RATE
#define CONTROL_FREQUENCY CONTROL_RATE
#pragma message "CONTROL_RATE is deprecated. Please use CONTROL_FREQUENCY instead. Defaults to 60Hz if not defined."
#else
#define CONTROL_FREQUENCY 60
#endif  // CONTROL_RATE
#endif  // CONTROL_FREQUENCY

#ifndef NAVIGATION_FREQUENCY
#define NAVIGATION_FREQUENCY 4
#endif

#ifndef MODULES_FREQUENCY
#define MODULES_FREQUENCY 60
#endif

#endif /* AUTOPILOT_H */
