/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

/**
 * @file subsystems/ins/hf_float.h
 *
 * Horizontal filter (x,y) to estimate position and velocity.
 *
 */

#ifndef HF_FLOAT_H
#define HF_FLOAT_H

#include "std.h"
#include "math/pprz_algebra_float.h"
#include "generated/airframe.h"

#define HFF_STATE_SIZE 2

#ifndef AHRS_PROPAGATE_FREQUENCY
#define AHRS_PROPAGATE_FREQUENCY PERIODIC_FREQUENCY
#endif

#ifndef HFF_PRESCALER
#if AHRS_PROPAGATE_FREQUENCY == 512
#define HFF_PRESCALER 16
#elif AHRS_PROPAGATE_FREQUENCY == 500
#define HFF_PRESCALER 10
#else
#error "HFF_PRESCALER needs to be a divisor of AHRS_PROPAGATE_FREQUENCY"
#endif
#endif

/* horizontal filter propagation frequency */
#define HFF_FREQ (AHRS_PROPAGATE_FREQUENCY/HFF_PRESCALER)
#define DT_HFILTER (1./HFF_FREQ)

#define HFF_UPDATE_SPEED

struct HfilterFloat {
  float x;
  /* float xbias; */
  float xdot;
  float xdotdot;
  float y;
  /* float ybias; */
  float ydot;
  float ydotdot;
  float xP[HFF_STATE_SIZE][HFF_STATE_SIZE];
  float yP[HFF_STATE_SIZE][HFF_STATE_SIZE];
  uint8_t lag_counter;
  bool_t rollback;
};

extern struct HfilterFloat b2_hff_state;

extern float b2_hff_x_meas;
extern float b2_hff_y_meas;
extern float b2_hff_xd_meas;
extern float b2_hff_yd_meas;
extern float b2_hff_xdd_meas;
extern float b2_hff_ydd_meas;

extern void b2_hff_init(float init_x, float init_xdot, float init_y, float init_ydot);
extern void b2_hff_propagate(void);
extern void b2_hff_update_gps(struct FloatVect2* pos_ned, struct FloatVect2* speed_ned);
extern void b2_hff_update_pos(struct FloatVect2 pos, struct FloatVect2 Rpos);
extern void b2_hff_update_vel(struct FloatVect2 vel, struct FloatVect2 Rvel);
extern void b2_hff_realign(struct FloatVect2 pos, struct FloatVect2 vel);

#define HFF_LOST_LIMIT 1000
extern uint16_t b2_hff_lost_limit;
extern uint16_t b2_hff_lost_counter;

extern void b2_hff_store_accel_body(void);

extern struct HfilterFloat *b2_hff_rb_last;
extern int lag_counter_err;
extern int save_counter;

#endif /* HF_FLOAT_H */
