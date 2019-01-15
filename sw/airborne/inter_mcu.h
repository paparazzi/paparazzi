/*
 * Copyright (C) 2003-2005  Pascal Brisset, Antoine Drouin
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

/**
 * @file inter_mcu.h
 * Communication between fbw and ap processes.
 *
 * This unit contains the data structure used to communicate between the
 * "fly by wire" process and the "autopilot" process. It must be linked once in a
 * monoprocessor architecture, twice in a twin-processors architecture.
 * In the latter case, the inter-mcu communication process (e.g. SPI) must fill and
 * read these data structures.
 */

#ifndef INTER_MCU_H
#define INTER_MCU_H


#ifdef INTER_MCU

#include <inttypes.h>

#include "std.h"

#include "paparazzi.h"
#include "pprz_mutex.h"
#include "generated/airframe.h"
#include "subsystems/radio_control.h"
#include "subsystems/electrical.h"
#include "firmwares/fixedwing/main_fbw.h"


/** Data structure shared by fbw and ap processes */
struct fbw_state {
#if defined RADIO_CONTROL || RADIO_CONTROL_AUTO1
  pprz_t channels[RADIO_CONTROL_NB_CHANNEL];
  uint8_t ppm_cpt;
#endif
  uint8_t status;
  uint8_t nb_err;
  struct Electrical electrical;
};

struct ap_state {
  pprz_t commands[COMMANDS_NB];
  pprz_t command_roll_trim;
  pprz_t command_pitch_trim;
  pprz_t command_yaw_trim;
};

// Status bits from FBW to AUTOPILOT
#define STATUS_RADIO_OK 0
#define STATUS_RADIO_REALLY_LOST 1
#define STATUS_MODE_AUTO 2
#define STATUS_MODE_FAILSAFE 3
#define AVERAGED_CHANNELS_SENT 4
#define MASK_FBW_CHANGED 0xf


extern struct fbw_state *fbw_state;
extern struct ap_state  *ap_state;

extern volatile bool inter_mcu_received_fbw;
extern volatile bool inter_mcu_received_ap;

/**
 * Getter and setter functions for ap_state and fbw_state elements
 * protected by mutexes
 */
PPRZ_MUTEX_DECL(ap_state_mtx);
PPRZ_MUTEX_DECL(fbw_state_mtx);

/** get AP command
 * @param cmd_idx command index
 * @return command value
 */
static inline pprz_t imcu_get_command(uint8_t cmd_idx)
{
  PPRZ_MUTEX_LOCK(ap_state_mtx);
  pprz_t val = ap_state->commands[cmd_idx];
  PPRZ_MUTEX_UNLOCK(ap_state_mtx);
  return val;
}

/** set AP command
 * @param cmd_idx command index
 * @param cmd new command value
 */
static inline void imcu_set_command(uint8_t cmd_idx, pprz_t cmd)
{
  PPRZ_MUTEX_LOCK(ap_state_mtx);
  ap_state->commands[cmd_idx] = cmd;
  PPRZ_MUTEX_UNLOCK(ap_state_mtx);
}

/** get roll trim
 * @return roll trim value
 */
static inline pprz_t imcu_get_roll_trim(void)
{
  PPRZ_MUTEX_LOCK(ap_state_mtx);
  pprz_t val = ap_state->command_roll_trim;
  PPRZ_MUTEX_UNLOCK(ap_state_mtx);
  return val;
}

/** set roll trim
 * @param roll_trim new roll trim value
 */
static inline void imcu_set_roll_trim(pprz_t roll_trim)
{
  PPRZ_MUTEX_LOCK(ap_state_mtx);
  ap_state->command_roll_trim = roll_trim;
  PPRZ_MUTEX_UNLOCK(ap_state_mtx);
}

/** get pitch trim
 * @return pitch trim value
 */
static inline pprz_t imcu_get_pitch_trim(void)
{
  PPRZ_MUTEX_LOCK(ap_state_mtx);
  pprz_t val = ap_state->command_pitch_trim;
  PPRZ_MUTEX_UNLOCK(ap_state_mtx);
  return val;
}

/** set pitch trim
 * @param pitch_trim new pitch trim value
 */
static inline void imcu_set_pitch_trim(pprz_t pitch_trim)
{
  PPRZ_MUTEX_LOCK(ap_state_mtx);
  ap_state->command_pitch_trim = pitch_trim;
  PPRZ_MUTEX_UNLOCK(ap_state_mtx);
}

/** get yaw trim
 * @return yaw trim value
 */
static inline pprz_t imcu_get_yaw_trim(void)
{
  PPRZ_MUTEX_LOCK(ap_state_mtx);
  pprz_t val = ap_state->command_yaw_trim;
  PPRZ_MUTEX_UNLOCK(ap_state_mtx);
  return val;
}

/** set yaw trim
 * @param yaw_trim new yaw trim value
 */
static inline void imcu_set_yaw_trim(pprz_t yaw_trim)
{
  PPRZ_MUTEX_LOCK(ap_state_mtx);
  ap_state->command_yaw_trim = yaw_trim;
  PPRZ_MUTEX_UNLOCK(ap_state_mtx);
}

#if defined RADIO_CONTROL || RADIO_CONTROL_AUTO1
/** get radio channel value
 * @param radio_idx radio channel index
 * @return radio channel value
 */
static inline pprz_t imcu_get_radio(uint8_t radio_idx)
{
  PPRZ_MUTEX_LOCK(fbw_state_mtx);
  pprz_t val = fbw_state->channels[radio_idx];
  PPRZ_MUTEX_UNLOCK(fbw_state_mtx);
  return val;
}

/** set radio channel
 * @param radio_idx radio index
 * @param radio new radio channel value
 */
static inline void imcu_set_radio(uint8_t radio_idx, pprz_t radio)
{
  PPRZ_MUTEX_LOCK(fbw_state_mtx);
  fbw_state->channels[radio_idx] = radio;
  PPRZ_MUTEX_UNLOCK(fbw_state_mtx);
}

/** get ppm count value
 * @return ppm count
 */
static inline uint8_t imcu_get_ppm_cpt(void)
{
  PPRZ_MUTEX_LOCK(fbw_state_mtx);
  uint8_t val = fbw_state->ppm_cpt;
  PPRZ_MUTEX_UNLOCK(fbw_state_mtx);
  return val;
}

/** set ppm count
 * @param ppm_cpt ppm counter
 */
static inline void imcu_set_ppm_cpt(uint8_t ppm_cpt)
{
  PPRZ_MUTEX_LOCK(fbw_state_mtx);
  fbw_state->ppm_cpt = ppm_cpt;
  PPRZ_MUTEX_UNLOCK(fbw_state_mtx);
}
#endif

/** get FBW status
 * @return status
 */
static inline uint8_t imcu_get_status(void)
{
  PPRZ_MUTEX_LOCK(fbw_state_mtx);
  uint8_t val = fbw_state->status;
  PPRZ_MUTEX_UNLOCK(fbw_state_mtx);
  return val;
}

/** set FBW status
 * @param status FBW status
 */
static inline void imcu_set_status(uint8_t status)
{
  PPRZ_MUTEX_LOCK(fbw_state_mtx);
  fbw_state->status = status;
  PPRZ_MUTEX_UNLOCK(fbw_state_mtx);
}

/** get electrical parameters
 * @param _electrical pointer to electrical data to fill
 */
static inline void imcu_get_electrical(struct Electrical *_electrical)
{
  PPRZ_MUTEX_LOCK(fbw_state_mtx);
  *_electrical = fbw_state->electrical;
  PPRZ_MUTEX_UNLOCK(fbw_state_mtx);
}

/** set electrical parameters
 * @param _electrical pointer to electrical data to use
 */
static inline void imcu_set_electrical(struct Electrical *_electrical)
{
  PPRZ_MUTEX_LOCK(fbw_state_mtx);
  fbw_state->electrical = *_electrical;
  PPRZ_MUTEX_UNLOCK(fbw_state_mtx);
}

#ifdef FBW

extern uint8_t time_since_last_ap;
extern bool ap_ok;

#define AP_STALLED_TIME        30  // 500ms with a 60Hz timer


static inline void inter_mcu_init(void)
{
  fbw_state->status = 0;
  fbw_state->nb_err = 0;

  PPRZ_MUTEX_INIT(ap_state_mtx);
  PPRZ_MUTEX_INIT(fbw_state_mtx);

  ap_ok = false;
}


/* Prepare data to be sent to mcu0 */
static inline void inter_mcu_fill_fbw_state(void)
{
  PPRZ_MUTEX_LOCK(fbw_state_mtx);
  uint8_t status = 0;

#ifdef RADIO_CONTROL
  uint8_t i;
  for (i = 0; i < RADIO_CONTROL_NB_CHANNEL; i++) {
    fbw_state->channels[i] = radio_control.values[i];
  }

  fbw_state->ppm_cpt = radio_control.frame_rate;

  status = (radio_control.status == RC_OK ? _BV(STATUS_RADIO_OK) : 0);
  status |= (radio_control.status == RC_REALLY_LOST ? _BV(STATUS_RADIO_REALLY_LOST) : 0);
  status |= (radio_control.status == RC_OK ? _BV(AVERAGED_CHANNELS_SENT) :
             0); // Any valid frame contains averaged channels
#endif // RADIO_CONTROL

  status |= (fbw_mode == FBW_MODE_AUTO ? _BV(STATUS_MODE_AUTO) : 0);
  status |= (fbw_mode == FBW_MODE_FAILSAFE ? _BV(STATUS_MODE_FAILSAFE) : 0);
  fbw_state->status  = status;

  fbw_state->electrical = electrical;
#if defined SINGLE_MCU
  /**Directly set the flag indicating to AP that shared buffer is available*/
  inter_mcu_received_fbw = true;
#endif
  PPRZ_MUTEX_UNLOCK(fbw_state_mtx);
}

/** Prepares date for next comm with AP. Set ::ap_ok to TRUE */
static inline void inter_mcu_event_task(void)
{
  time_since_last_ap = 0;
  ap_ok = true;
}

/** Monitors AP. Set ::ap_ok to false if AP is down for a long time. */
static inline void inter_mcu_periodic_task(void)
{
  if (time_since_last_ap >= AP_STALLED_TIME) {
    ap_ok = false;
#ifdef SINGLE_MCU
    // Keep filling the buffer even if no AP commands are received
    inter_mcu_fill_fbw_state();
#endif

  } else {
    time_since_last_ap++;
  }
}

#endif /* FBW */

#endif /* INTER_MCU */

#endif /* INTER_MCU_H */
