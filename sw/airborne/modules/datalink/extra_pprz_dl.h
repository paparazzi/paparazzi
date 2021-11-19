/*
 * Copyright (C) 2010  ENAC
 * Copyright (C) 2016  2016 Michal Podhradsky <http://github.com/podhrmic>
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
/**
 * @file "modules/datalink/extra_pprz_dl.h"
 * Extra datalink and telemetry using PPRZ protocol
 */
#ifndef EXTRA_PPRZ_DL_H
#define EXTRA_PPRZ_DL_H

#include "modules/datalink/datalink.h"
#include "pprzlink/pprz_transport.h"

#if USE_UDP
#include "mcu_periph/udp.h"
#endif

#if USE_USB_SERIAL
#include "mcu_periph/usb_serial.h"
#endif

/* PPRZ transport structure */
extern struct pprz_transport extra_pprz_tp;

extern uint8_t extra_dl_buffer[MSG_SIZE]  __attribute__((aligned));

/** Datalink Event */
void extra_pprz_dl_event(void);


/** Init function */
extern void extra_pprz_dl_init(void);

/** Periodic function
 *
 * should be called at TELEMETRY_FREQUENCY
 */
extern void extra_pprz_dl_periodic(void);

#endif /* EXTRA_PPRZ_DL_H */

