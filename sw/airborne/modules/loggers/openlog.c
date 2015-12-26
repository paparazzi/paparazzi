/*
 * Copyright (C) 2011 Christoph Niemann
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
 * @file openlog.c
 *
 * This module provides a timestamp-message, allowing
 * sw/logalizer/openlog2tlm to convert a recorded dumpfile,
 * created by openlog into the pprz-tlm format, to be converted into
 * .data and .log files by sw/logalizer/sd2log
 */

#include "openlog.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
#include "mcu_periph/sys_time.h"

void periodic_2Hz_openlog(void)
{
  uint32_t timestamp = get_sys_time_msec();
  DOWNLINK_SEND_TIMESTAMP(DefaultChannel, DefaultDevice, &timestamp);
}
