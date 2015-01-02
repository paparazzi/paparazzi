/*
 * Copyright (C) 2010 ENAC
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

/** \file tcas.h
 *  \brief Collision avoidance library
 *
 */

#ifndef TCAS_H
#define TCAS_H

#include "std.h"
#include "subsystems/navigation/traffic_info.h"

extern float tcas_alt_setpoint;
extern float tcas_tau_ta, tcas_tau_ra, tcas_dmod, tcas_alim;

#define TCAS_NO_ALARM 0
#define TCAS_TA 1
#define TCAS_RA 2
enum tcas_resolve { RA_NONE, RA_LEVEL, RA_CLIMB, RA_DESCEND };

extern uint8_t tcas_status;
extern enum tcas_resolve tcas_resolve;
extern uint8_t tcas_ac_RA;

struct tcas_ac_status {
  uint8_t status;
  enum tcas_resolve resolve;
};

extern struct tcas_ac_status tcas_acs_status[NB_ACS];

extern void tcas_init(void);
extern void tcas_periodic_task_1Hz(void);
extern void tcas_periodic_task_4Hz(void);

#define CallTCAS() { if (tcas_status == TCAS_RA) v_ctl_altitude_setpoint = tcas_alt_setpoint; }

#define ParseTcasResolve() { \
    if (DL_TCAS_RESOLVE_ac_id(dl_buffer) == AC_ID) { \
      uint8_t ac_id_conflict = DL_TCAS_RESOLVE_ac_id_conflict(dl_buffer); \
      tcas_acs_status[the_acs_id[ac_id_conflict]].resolve = DL_TCAS_RESOLVE_resolve(dl_buffer); \
    } \
  }

#endif // TCAS
