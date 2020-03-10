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

#include "modules/datalink/pprz_dl.h"
#include "subsystems/datalink/datalink.h"

#ifndef PPRZ_UPDATE_DL
#define PPRZ_UPDATE_DL TRUE
#endif

struct pprz_transport pprz_tp;

void pprz_dl_init(void)
{
  pprz_transport_init(&pprz_tp);
}

void pprz_dl_event(void)
{
  pprz_check_and_parse(&DOWNLINK_DEVICE.device, &pprz_tp, dl_buffer, &dl_msg_available);
  DlCheckAndParse(&DOWNLINK_DEVICE.device, &pprz_tp.trans_tx, dl_buffer, &dl_msg_available, PPRZ_UPDATE_DL);
}

