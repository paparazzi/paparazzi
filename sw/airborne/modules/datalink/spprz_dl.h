/*
 * Copyright (C) 2017 Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
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

/** \file modules/datalink/spprz_dl.h
 *  \brief Datalink using secure PPRZ protocol
 */

#ifndef SPPRZ_DL_H
#define SPPRZ_DL_H

#include "pprzlink/spprz_transport.h"

#include "mcu_periph/uart.h"
#if USE_USB_SERIAL
#include "mcu_periph/usb_serial.h"
#endif
#if USE_UDP
#include "mcu_periph/udp.h"
#endif

/** PPRZ transport structure */
extern struct spprz_transport spprz_tp;

/** Init function */
extern void spprz_dl_init(void);

/** Datalink Event */
extern void spprz_dl_event(void);

#endif /* SPPRZ_DL_H */

