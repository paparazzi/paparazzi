/*
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
 * Copyright (C) 2012 The Paparazzi Team
 * Copyright (C) 2016 Michal Podhradsky <http://github.com/podhrmic>
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
#ifndef NPS_INS_H
#define NPS_INS_H

#include "std.h"
#include "nps_fdm.h"

// if undefined, match with control frequency because that is how it should be used
#ifndef INS_FREQUENCY
#ifdef CONTROL_FREQUENCY
#define INS_FREQUENCY CONTROL_FREQUENCY
#else
#define INS_FREQUENCY PERIODIC_FREQUENCY
#endif
#endif

extern uint8_t *ins_buffer;

extern void nps_ins_init(void);
void nps_ins_fetch_data(struct NpsFdm *fdm_ins);
uint16_t nps_ins_fill_buffer(void);

#endif /* NPS_INS_H */
