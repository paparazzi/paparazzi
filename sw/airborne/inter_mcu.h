/*  $Id$
 *
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.  
 *
 */

/** \brief Communication between fbw and ap processes
 * This unit contains the data structure used to communicate between the
 * "fly by wire" process and the "autopilot" process. It must be linked once in a
 * monoprocessor architecture, twice in a twin-processors (the historical
 * Atmel AVRs mega8-mega128 one) architecture. In the latter case, the
 * inter-mcu communication process (e.g. SPI) must fill and read these data structures.
*/

#ifndef INTER_MCU_H
#define INTER_MCU_H

/** Fly by wire modes */
#define FBW_MODE_MANUAL   0
#define FBW_MODE_AUTO     1
#define FBW_MODE_FAILSAFE 2
#define FBW_MODE_OF_PPRZ(mode) ((mode) < TRESHOLD_MANUAL_PPRZ ? FBW_MODE_MANUAL : FBW_MODE_AUTO)

#ifdef INTER_MCU

#include <inttypes.h>

#include "std.h"
#include "radio.h"
#include "paparazzi.h"
#include "airframe.h"
#include "radio_control.h"
#include "main_fbw.h"

#ifdef SITL
#include <stdio.h>
#endif

/*
 * System clock in MHz.
 */
#define CLOCK		16

/* !!!!!!!!!!!!!!!!!!! Value used in gen_airframe.ml !!!!!!!!!!!!!!!!! */

/** Data structure shared by fbw and ap sub-mofules */
struct from_fbw_msg {
  pprz_t channels[RADIO_CTL_NB];  
  uint8_t ppm_cpt;
  uint8_t status;
  uint8_t nb_err;
  uint8_t vsupply; /* 1e-1 V */
#if defined IMU_ANALOG || defined IMU_3DMG
  int16_t euler_dot[3];
#endif
#ifdef IMU_3DMG
  int16_t euler[3];
#endif
};

struct from_ap_msg {
  pprz_t commands[COMMANDS_NB];  
};

typedef union  { 
  struct from_fbw_msg from_fbw; 
  struct from_ap_msg from_ap;
} inter_mcu_msg;


// Status bits from FBW to AUTOPILOT
#define STATUS_RADIO_OK 0
#define RADIO_REALLY_LOST 1
#define STATUS_MODE_AUTO 2
#define STATUS_MODE_FAILSAFE 3
#define AVERAGED_CHANNELS_SENT 4
#define MASK_FBW_CHANGED 0xf

// Statut bits from AUTOPILOT to FBW
#define STATUS_AUTO_OK  0

/** Two bytes for the CRC */
#define FRAME_LENGTH (sizeof(inter_mcu_msg)+2)

#define CRC_INIT 0xffff
#define CrcUpdate(_crc, _data) _crc_ccitt_update(_crc, _data)
#define Crc1(x) ((x)&0xff)
#define Crc2(x) ((x)>>8)

#define TRESHOLD_MANUAL_PPRZ (MIN_PPRZ / 2)

extern inter_mcu_msg from_fbw;
extern inter_mcu_msg from_ap;

extern volatile bool_t from_fbw_receive_valid;
extern volatile bool_t from_ap_receive_valid;

extern uint8_t time_since_last_ap;
extern bool_t ap_ok;


#ifdef FBW

#define AP_STALLED_TIME        30  // 500ms with a 60Hz timer


/* Prepare data to be sent to mcu0 */
static inline void to_autopilot_from_rc_values (void) {
  struct from_fbw_msg *msg = &(from_fbw.from_fbw);

  uint8_t i;
  for(i = 0; i < RADIO_CTL_NB; i++)
    msg->channels[i] = rc_values[i];

  uint8_t status;
  status = (rc_status == RC_OK ? _BV(STATUS_RADIO_OK) : 0);
  status |= (rc_status == RC_REALLY_LOST ? _BV(RADIO_REALLY_LOST) : 0);
  status |= (fbw_mode == FBW_MODE_AUTO ? _BV(STATUS_MODE_AUTO) : 0);
  status |= (fbw_mode == FBW_MODE_FAILSAFE ? _BV(STATUS_MODE_FAILSAFE) : 0);
  msg->status  = status;

  if (rc_values_contains_avg_channels) {
    msg->status |= _BV(AVERAGED_CHANNELS_SENT);
    rc_values_contains_avg_channels = FALSE;
  }
  msg->ppm_cpt = last_ppm_cpt;
  msg->vsupply = fbw_vsupply_decivolt;
#if defined IMU_3DMG || defined IMU_ANALOG
  msg->euler_dot[0] = roll_dot;
  msg->euler_dot[1] = pitch_dot;
  msg->euler_dot[2] = yaw_dot;
#endif
#ifdef IMU_3DMG
  msg->euler[0] = roll;
  msg->euler[1] = pitch;
  msg->euler[2] = yaw;
#endif
}

/** Prepares date for next comm with AP. Set ::ap_ok to TRUE */
static inline void inter_mcu_event_task( void) {
  time_since_last_ap = 0;
  ap_ok = TRUE;
  to_autopilot_from_rc_values();
#if defined AP
  /**Directly set the flag indicating to AP that shared buffer is available*/
  from_fbw_receive_valid = TRUE;
#endif
}

/** Monitors AP. Set ::rc_ok to false if AP is down for a long time. */
static inline void inter_mcu_periodic_task(void) {
  if (time_since_last_ap >= AP_STALLED_TIME) {
    ap_ok = FALSE;
  } else
    time_since_last_ap++;
}

#endif /* FBW */

#endif /* INTER_MCU */

#endif /* INTER_MCU_H */
