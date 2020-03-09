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
  float k1;
  float k2;
  float alpha;
  uint16_t timeout;
  float p_centroid_x;
  float p_centroid_y;
  float v_centroid_x;
  float v_centroid_y;
  float target_px;
  float target_py;
  float target_vx;
  float target_vy;
  float vx;
  float vy;
  float px;
  float py;
  float ref_px;
  float ref_py;
  float omega;
  uint16_t time_broad;
} ctc_con;

extern ctc_con ctc_control;
extern int16_t tableNei[][6];
extern float ctc_error_to_target;

extern void ctc_init(void);
extern bool collective_tracking_vehicle(void);
extern bool collective_tracking_waypoint(uint8_t wp);
extern bool collective_tracking_point(float x, float y);
extern void collective_tracking_control(void);
extern void ctc_send_info_to_nei(void);

extern void parse_ctc_RegTable(uint8_t *buf);
extern void parse_ctc_CleanTable(uint8_t *buf);
extern void parse_ctc_NeiInfoTable(uint8_t *buf);
extern void parse_ctc_TargetInfo(uint8_t *buf);

#endif // CTC_H
