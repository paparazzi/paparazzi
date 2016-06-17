/*
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
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

/** @file subsystems/intermcu/intermcu_ap.h
 *  @brief Rotorcraft Inter-MCU on the autopilot
 */

#ifndef INTERMCU_AP_ROTORCRAFT_H
#define INTERMCU_AP_ROTORCRAFT_H

#include "subsystems/intermcu.h"
#include "generated/airframe.h"

void intermcu_set_actuators(pprz_t *command_values, uint8_t ap_mode);
void RadioControlEvent(void (*frame_handler)(void));
void intermcu_send_spektrum_bind(void);
void intermcu_set_enabled(bool value);

/* We need radio defines for the Autopilot */
#define RADIO_THROTTLE   0
#define RADIO_ROLL       1
#define RADIO_PITCH      2
#define RADIO_YAW        3
#define RADIO_MODE       4
#define RADIO_KILL_SWITCH 5
#define RADIO_AUX1       5
#define RADIO_AUX2       6
#define RADIO_AUX3       7
#define RADIO_CONTROL_NB_CHANNEL 8

/* Structure for FBW status */
struct fbw_status_t {
  uint8_t rc_status;
  uint8_t frame_rate;
  uint8_t mode;
  uint16_t vsupply;
  int32_t current;
};

#endif /* INTERMCU_AP_ROTORCRAFT_H */
