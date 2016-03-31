/*
 * Copyright (C) 2010 Martin Mueller
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

/** \file windturbine.c
 *
 *   This measures a trigger pulse length (e.g. duration of a wind turbine
 *   rotation) and sends a message with the info.
 */


#include "meteo/windturbine.h"
#include "core/trigger_ext.h"
#include "subsystems/gps.h"
#include "mcu_periph/sys_time.h"


#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"


void windturbine_periodic(void)
{
  if (trigger_ext_valid == TRUE) {
    uint8_t ac_id = 0;
    uint8_t turb_id = TURBINE_ID;
    uint32_t sync_itow, cycle_time;

    sync_itow = gps_tow_from_sys_ticks(trigger_t0);
    cycle_time = msec_of_sys_time_ticks(trigger_delta_t0);

    DOWNLINK_SEND_WINDTURBINE_STATUS_(DefaultChannel, DefaultDevice,
                                      &ac_id,
                                      &turb_id,
                                      &sync_itow,
                                      &cycle_time);
    trigger_ext_valid = false;
  }
}

