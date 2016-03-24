/*
 * Copyright (C) 2003-2005  Pascal Brisset, Antoine Drouin
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
 */

/**
 * @file inter_mcu.c
 * Communication between fbw and ap processes.
 *
 */

#include "inter_mcu.h"

#if defined SINGLE_MCU
static struct fbw_state _fbw_state;
static struct ap_state _ap_state;
struct fbw_state *fbw_state = &_fbw_state;
struct ap_state *ap_state = &_ap_state;
#else /* SINGLE_MCU */
#include "link_mcu.h"
struct fbw_state *fbw_state = &link_mcu_from_fbw_msg.payload.from_fbw;
struct ap_state  *ap_state = &link_mcu_from_ap_msg.payload.from_ap;
#endif /* ! SINGLE_MCU */

volatile bool inter_mcu_received_fbw = false;
volatile bool inter_mcu_received_ap  = false;

#ifdef FBW
/** Variables for monitoring AP communication status */
bool ap_ok;
uint8_t time_since_last_ap;
#endif
