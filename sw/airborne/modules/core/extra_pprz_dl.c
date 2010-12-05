/*
 * Paparazzi mcu0 $Id: pprz_transport.c 929 2006-06-02 12:11:37Z poine $
 *
 * Copyright (C) 2006  Pascal Brisset, Antoine Drouin
 * Copyright (C) 2010  ENAC
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

#include <inttypes.h>
#include "extra_pprz_dl.h"
#include "mcu_periph/uart.h"

volatile bool_t extra_pprz_msg_received = FALSE;
uint8_t extra_pprz_ovrn, extra_pprz_error;
volatile uint8_t extra_pprz_payload_len;
uint8_t extra_pprz_payload[PPRZ_PAYLOAD_LEN];
