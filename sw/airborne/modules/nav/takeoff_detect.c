/*
 * Copyright (C) 2014 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi

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
 *
 */

/**
 * @file modules/nav/takeoff_detect.c
 *
 * Automatic takeoff assistance for fixed-wing.
 * The planes's launching can be triggered/aborted
 * by pointing the nose up or down for a given time.
 */

#include "std.h"
#include "modules/nav/takeoff_detect.h"
#include "firmwares/fixedwing/autopilot.h"
#include "state.h"
#include "generated/modules.h" // for status and function's period

/** Default pitch angle to trigger launch */
#ifndef TAKEOFF_DETECT_LAUNCH_PITCH
#define TAKEOFF_DETECT_LAUNCH_PITCH RadOfDeg(30.)
#endif

/** Default pitch angle to cancel launch */
#ifndef TAKEOFF_DETECT_ABORT_PITCH
#define TAKEOFF_DETECT_ABORT_PITCH RadOfDeg(-20.)
#endif

/** Detection timer in seconds */
#ifndef TAKEOFF_DETECT_TIMER
#define TAKEOFF_DETECT_TIMER 2.
#endif

/** Disable timer in seconds */
#ifndef TAKEOFF_DETECT_DISABLE_TIMER
#define TAKEOFF_DETECT_DISABLE_TIMER 4.
#endif

/** Takeoff detection states */
enum takeoff_detect_state {
  TO_DETECT_DISABLED,
  TO_DETECT_ARMED,
  TO_DETECT_LAUNCHING
};

/** Takeoff detection structure */
struct takeoff_detect_struct {
  enum takeoff_detect_state state;
  uint32_t timer;
};

static struct takeoff_detect_struct takeoff_detect;

// Init
void takeoff_detect_init(void)
{
  // variable init is done in start function
}

// Start
void takeoff_detect_start(void)
{
  takeoff_detect.state = TO_DETECT_ARMED; // always start periodic with ARMED state
  takeoff_detect.timer = 0; // and reset timer
}

// Periodic
void takeoff_detect_periodic(void)
{
  // Run detection state machine here
  switch (takeoff_detect.state) {
    case TO_DETECT_ARMED:
      // test for "nose up" + AP in AUTO2 (+ GPS OK ? FIXME)
      if (stateGetNedToBodyEulers_f()->theta > TAKEOFF_DETECT_LAUNCH_PITCH
          && pprz_mode == PPRZ_MODE_AUTO2) {
        takeoff_detect.timer++;
      } else {
        // else reset timer
        takeoff_detect.timer = 0;
      }
      // if timer is finished, start launching
      if (takeoff_detect.timer > (int)(TAKEOFF_DETECT_PERIODIC_FREQ * TAKEOFF_DETECT_TIMER)) {
        launch = true;
        takeoff_detect.state = TO_DETECT_LAUNCHING;
        takeoff_detect.timer = 0;
      }
      break;
    case TO_DETECT_LAUNCHING:
      // abort if pitch goes below threshold while launching
      if (stateGetNedToBodyEulers_f()->theta < TAKEOFF_DETECT_ABORT_PITCH
          || pprz_mode != PPRZ_MODE_AUTO2) {
        // back to ARMED state
        launch = false;
        takeoff_detect.state = TO_DETECT_ARMED;
      }
      // increment timer and disable detection after some time
      takeoff_detect.timer++;
      if (takeoff_detect.timer > (int)(TAKEOFF_DETECT_PERIODIC_FREQ * TAKEOFF_DETECT_DISABLE_TIMER)) {
        takeoff_detect.state = TO_DETECT_DISABLED;
      }
      break;
    case TO_DETECT_DISABLED:
      // stop periodic call
      takeoff_detect_takeoff_detect_periodic_status = MODULES_STOP;
      break;
    default:
      // No kidding ?!
      takeoff_detect.state = TO_DETECT_DISABLED;
      break;
  }
}


