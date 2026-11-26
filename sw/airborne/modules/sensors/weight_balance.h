/*
 * Copyright (C) 2026 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi
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

/** @file modules/sensors/weight_balance.h
 * Weight and center of gravity from load cells in the landing gear
 */

#ifndef WEIGHT_BALANCE_H
#define WEIGHT_BALANCE_H

#include "std.h"

#define WEIGHT_BALANCE_MAX_WHEELS 4  ///< also the array length of force and raw in the WEIGHT_BALANCE module message

struct weight_balance_wheel_t {
  uint8_t cell_id;          ///< Load cell id (from uavcan or the local ADS1220)
  float x;                  ///< Wheel contact point in body frame, forward [m]
  float y;                  ///< Wheel contact point in body frame, right [m]
  float offset;             ///< Load cell reading without load [counts]
  float scale;              ///< Load cell scale [N/count]
  float raw;                ///< Last filtered load cell reading [counts]
  float force;              ///< Calibrated vertical force [N]
  float last_time;          ///< Time of the last reading [s]
};

struct weight_balance_t {
  uint8_t nb_wheels;        ///< Number of wheels with a load cell
  struct weight_balance_wheel_t wheels[WEIGHT_BALANCE_MAX_WHEELS];
  float cg_height;          ///< Height of the CG above the wheel contact plane [m]

  bool valid;               ///< All wheels have recent data and carry load
  float mass;               ///< Total mass [kg]
  float cg_x;               ///< CG position in body frame, forward [m]
  float cg_y;               ///< CG position in body frame, right [m]

  uint8_t cal_wheel;        ///< Wheel index used by the calibration settings
};

extern struct weight_balance_t weight_balance;

extern void weight_balance_init(void);
extern void weight_balance_periodic(void);
extern void weight_balance_event(void);

/* Settings handlers */
extern void weight_balance_tare(float tare);
extern void weight_balance_calibrate(float mass);

#endif /* WEIGHT_BALANCE_H */
