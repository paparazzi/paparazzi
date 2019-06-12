/*
 * Copyright (C) Paparazzi team
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

#ifndef NPS_AUTOPILOT_H
#define NPS_AUTOPILOT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "generated/airframe.h"

#include "nps_radio_control.h"

/**
 * Number of commands sent to the FDM of NPS.
 * If MOTOR_MIXING_NB_MOTOR is defined (usually rotorcraft firmware)
 * we have that many commands (one per motor),
 * otherwise we default to the number of high level commands (COMMANDS_NB).
 */
#ifndef NPS_COMMANDS_NB
#if defined MOTOR_MIXING_NB_MOTOR
#define NPS_COMMANDS_NB MOTOR_MIXING_NB_MOTOR
#else
#define NPS_COMMANDS_NB COMMANDS_NB
#endif /* #if defined MOTOR_MIXING_NB_MOTOR */
#endif /* #ifndef NPS_COMMANDS_NB */

struct NpsAutopilot {
  double commands[NPS_COMMANDS_NB];
  bool launch;
};

extern struct NpsAutopilot nps_autopilot;

extern bool nps_bypass_ahrs;
extern bool nps_bypass_ins;
extern void sim_overwrite_ahrs(void);
extern void sim_overwrite_ins(void);

extern void nps_autopilot_init(enum NpsRadioControlType type, int num_script, char *js_dev);
extern void nps_autopilot_run_step(double time);
extern void nps_autopilot_run_systime_step(void);

#ifdef __cplusplus
}
#endif

#endif /* NPS_AUTOPILOT_H */
