/*
 * Copyright (C) 2017  Hector Garcia de Marina
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

/** @file dcf.h
 *
 *  Distributed circular formation algorithm
 */

#ifndef DCF_H
#define DCF_H

#include "std.h"

/*! Default number of neighbors per aircraft */
#ifndef DCF_MAX_NEIGHBORS
#define DCF_MAX_NEIGHBORS 4
#endif

struct dcf_con {
  float k;
  float radius;
  uint16_t timeout;
  float theta;
  uint16_t broadtime;
};

extern struct dcf_con dcf_control;

struct dcf_tab {
  int16_t tableNei[DCF_MAX_NEIGHBORS][4];
  int16_t error_sigma[DCF_MAX_NEIGHBORS];
  uint32_t last_theta[DCF_MAX_NEIGHBORS];
};

extern struct dcf_tab dcf_tables;

extern void dcf_init(void);
extern bool distributed_circular(uint8_t wp);
extern void send_theta_to_nei(void);

extern void parseRegTable(void);
extern void parseThetaTable(void);

#endif // DCF_H
