/*
 * Paparazzi $Id$
 *
 * Copyright (C) 2005 Pascal Brisset, Antoine Drouin
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

/** \file main_ap.h
 *  \brief AP ( AutoPilot ) process API
 *
 */

#ifndef AP_H
#define AP_H

#include "mcu_periph/sys_time.h"

extern tid_t ap_periodic_tid; ///< id for periodic_task_ap() timer
extern tid_t telemetry_tid;   ///< id for telemetry_periodic() timer
extern tid_t sensors_tid;     ///< id for sensors_task() timer
extern tid_t attitude_tid;    ///< id for attitude_loop() timer
extern tid_t navigation_tid;  ///< id for navigation_task() timer
extern tid_t monitor_tid;     ///< id for monitor_task() timer


extern void init_ap( void );
extern void periodic_task_ap( void );
extern void event_task_ap( void );
extern void reporting_task( void );
extern void sensors_task( void );
extern void attitude_loop( void );
extern void navigation_task(void);
extern void monitor_task(void);

#endif
