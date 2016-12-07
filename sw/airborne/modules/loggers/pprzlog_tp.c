/*
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/** \file modules/loggers/pprzlog_tp.c
 *  \brief Initialize pprzlog transport
 */

#include "modules/loggers/pprzlog_tp.h"
#include "mcu_periph/sys_time.h"

struct pprzlog_transport pprzlog_tp;

void pprzlog_tp_init(void)
{
  pprzlog_transport_init(&pprzlog_tp, get_sys_time_usec);
}


