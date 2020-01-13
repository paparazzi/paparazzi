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

/** \file modules/datalink/pprz_dl.h
 *  \brief Datalink using PPRZ protocol
 */

#ifndef PPRZ_DL_H
#define PPRZ_DL_H

#include "pprzlink/pprz_transport.h"

#include "mcu_periph/uart.h"
#if USE_USB_SERIAL
#include "mcu_periph/usb_serial.h"
#endif
#if USE_UDP
#include "mcu_periph/udp.h"
#endif
#if USE_SYSLINK
#include "modules/datalink/bitcraze/syslink_dl.h"
#endif

/** PPRZ transport structure */
extern struct pprz_transport pprz_tp;

/** Init function */
extern void pprz_dl_init(void);

/** Datalink Event */
extern void pprz_dl_event(void);

#endif /* PPRZ_DL_H */

