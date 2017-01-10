/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 * Copyright (C) 2016 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file firmwares/rotorcraft/autopilot.h
 *
 * Core autopilot file.
 * Using either static or generated autopilot logic.
 *
 */

#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include "std.h"
#include "generated/airframe.h"
#include "state.h"
#include "firmwares/rotorcraft/autopilot_utils.h"

// include static or generated autopilot
// static version by default
#ifndef USE_GENERATED_AUTOPILOT
#define USE_GENERATED_AUTOPILOT FALSE
#endif

#if USE_GENERATED_AUTOPILOT
#include "firmwares/rotorcraft/autopilot_generated.h"
#else
#include "firmwares/rotorcraft/autopilot_static.h"
#endif

extern uint8_t autopilot_mode;
extern uint8_t autopilot_mode_auto2;
extern bool autopilot_motors_on;
extern bool autopilot_in_flight;
extern bool kill_throttle;
extern bool autopilot_rc;
extern uint32_t autopilot_in_flight_counter;

extern bool autopilot_power_switch;

extern void autopilot_init(void);
extern void autopilot_periodic(void);
extern void autopilot_on_rc_frame(void);
extern void autopilot_set_mode(uint8_t new_autopilot_mode);
extern void autopilot_SetModeHandler(float new_autopilot_mode); // handler for dl_setting
extern void autopilot_set_motors_on(bool motors_on);
extern void autopilot_check_in_flight(bool motors_on);

extern bool autopilot_ground_detected;
extern bool autopilot_detect_ground_once;

extern uint16_t autopilot_flight_time;

#define autopilot_KillThrottle(_kill) { \
    if (_kill)                          \
      autopilot_set_motors_on(false);   \
    else                                \
      autopilot_set_motors_on(true);    \
  }

#ifdef POWER_SWITCH_GPIO
#include "mcu_periph/gpio.h"
#define autopilot_SetPowerSwitch(_v) {                          \
    autopilot_power_switch = _v;                                \
    if (_v) { gpio_set(POWER_SWITCH_GPIO); }  \
    else { gpio_clear(POWER_SWITCH_GPIO); }   \
  }
#else
#define autopilot_SetPowerSwitch(_v) {  \
    autopilot_power_switch = _v;        \
  }
#endif

/** Z-acceleration threshold to detect ground in m/s^2 */
#ifndef THRESHOLD_GROUND_DETECT
#define THRESHOLD_GROUND_DETECT 25.0
#endif
/** Ground detection based on vertical acceleration.
 */
static inline void DetectGroundEvent(void)
{
  if (autopilot_detect_ground_once
#ifdef AP_MODE_FAILSAFE
      || autopilot_mode == AP_MODE_FAILSAFE
#endif
     ) {
    struct NedCoor_f *accel = stateGetAccelNed_f();
    if (accel->z < -THRESHOLD_GROUND_DETECT ||
        accel->z > THRESHOLD_GROUND_DETECT) {
      autopilot_ground_detected = true;
      autopilot_detect_ground_once = false;
    }
  }
}

#include "subsystems/settings.h"

/* try to make sure that we don't write to flash while flying */
static inline void autopilot_StoreSettings(float store)
{
  if (kill_throttle && store) {
    settings_store_flag = store;
    settings_store();
  }
}

static inline void autopilot_ClearSettings(float clear)
{
  if (kill_throttle && clear) {
    settings_clear_flag = clear;
    settings_clear();
  }
}

#if DOWNLINK
#include "pprzlink/pprzlink_transport.h"
extern void send_autopilot_version(struct transport_tx *trans, struct link_device *dev);
#endif

#endif /* AUTOPILOT_H */
