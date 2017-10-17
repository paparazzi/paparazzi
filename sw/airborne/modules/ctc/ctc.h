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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file ctc.h
 *
 *  Collective Tracking Control
 */

#ifndef CTC_H
#define CTC_H

#include "std.h"

typedef struct {
  float k;
  uint16_t timeout;
  float speed_ref;
  float theta_ref;
  float speed;
  float theta;
  uint16_t time_broad;
} ctc_con;

extern ctc_con ctc_control;
extern int16_t tableNei[][4];

extern void ctc_init(void);
extern bool collective_tracking_control(void);
extern void send_theta_and_speed_to_nei(void);

extern void parse_ctc_RegTable(void);
extern void parse_ctc_CleanTable(void);
extern void parse_ctc_ThetaAndSpeedTable(void);

#endif // CTC_H
