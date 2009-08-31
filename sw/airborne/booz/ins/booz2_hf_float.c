/*
 * $Id$
 *
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 * Copyright (C) 2009 Felix Ruess <felix.ruess@gmail.com>
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

#include "booz2_hf_float.h"
#include "booz2_ins.h"
#include <stdlib.h>

/*

X_x = [ x xdot xbias ]
X_y = [ y ydot ybias ]


*/

/* horizontal filter propagation frequency */
#define HFF_FREQ 512./HFF_PRESCALER
#define DT_HFILTER 1./HFF_FREQ

/* initial covariance diagonal */
#define INIT_PXX 1.
#define INIT_PXX_BIAS 0.1
/* process noise (is the same for x and y)*/
#define ACCEL_NOISE 0.5
#define Q       ACCEL_NOISE*DT_HFILTER*DT_HFILTER/2.
#define Qdotdot ACCEL_NOISE*DT_HFILTER
#define Qbiasbias 1e-7*HFF_PRESCALER
//TODO: proper measurement noise
#define Rpos   5.
#define Rspeed 1.

/* output filter states */
struct hfilter_f b2_hff_state;
/* saved state when gps was valid */
struct hfilter_f b2_hff_save;
/* pointer to filter states to operate on */
struct hfilter_f *b2_hff_work;

/* float b2_hff_x; */
/* float b2_hff_xbias; */
/* float b2_hff_xdot; */
/* float b2_hff_xdotdot; */

/* float b2_hff_y; */
/* float b2_hff_ybias; */
/* float b2_hff_ydot; */
/* float b2_hff_ydotdot; */

/* filter covariance matrices */
/* float b2_hff_xP[B2_HFF_STATE_SIZE][B2_HFF_STATE_SIZE]; */
/* float b2_hff_yP[B2_HFF_STATE_SIZE][B2_HFF_STATE_SIZE]; */

#ifdef GPS_LAG
/* GPS_LAG is defined in seconds
 * GPS_LAG_PS in multiple of prescaler
 */
/* number of propagaton steps to redo according to GPS_LAG */
#define GPS_LAG_N (int) (GPS_LAG * 512. / HFF_PRESCALER + 0.5)

/* state and covariance when GPS was valid */
/* float b2_hff_x_sav; */
/* float b2_hff_xbias_sav; */
/* float b2_hff_xdot_sav; */
/* float b2_hff_xdotdot_sav; */
/* float b2_hff_y_sav; */
/* float b2_hff_ybias_sav; */
/* float b2_hff_ydot_sav; */
/* float b2_hff_ydotdot_sav; */
/* float b2_hff_xP_sav[B2_HFF_STATE_SIZE][B2_HFF_STATE_SIZE]; */
/* float b2_hff_yP_sav[B2_HFF_STATE_SIZE][B2_HFF_STATE_SIZE]; */

#define BUF_MAXN GPS_LAG_N+2
/* buffer with past mean accel values for redoing the propagation */
struct FloatVect2 past_accel[BUF_MAXN];

/* variables for accel buffer */
uint8_t buf_r; /* pos to read from, oldest measurement */
uint8_t buf_w; /* pos to write to */
uint8_t buf_n; /* number of elements in buffer */

/* number of propagation steps since state was saved */
/* uint8_t lag_counter; */
/* by how many steps the estimated GPS validity point in time differed from GPS_LAG_N */
int8_t lag_counter_err;

/* counts down the propagation steps until the filter state is saved again */
int8_t save_counter;

static inline void b2_hff_get_past_accel(int back_n);
static inline void b2_hff_rollback_filter(void);
static inline void b2_hff_save_filter(void);
#endif /* GPS_LAG */

/* acceleration used in propagation step */
float b2_hff_xaccel;
float b2_hff_yaccel;

/* last position measurement */
float b2_hff_x_meas;
float b2_hff_y_meas;


static inline void b2_hff_init_x(float init_x, float init_xdot, float init_xbias);
static inline void b2_hff_init_y(float init_y, float init_ydot, float init_ybias);

static inline void b2_hff_propagate_x(void);
static inline void b2_hff_propagate_y(void);

static inline void b2_hff_update_x(float x_meas);
static inline void b2_hff_update_y(float y_meas);

static inline void b2_hff_update_xdot(float v);
static inline void b2_hff_update_ydot(float v);



void b2_hff_init(float init_x, float init_xdot, float init_xbias, float init_y, float init_ydot, float init_ybias) {
  b2_hff_init_x(init_x, init_xdot, init_xbias);
  b2_hff_init_y(init_y, init_ydot, init_ybias);
  b2_hff_work = &b2_hff_state;
#ifdef GPS_LAG
  buf_r = 0;
  buf_w = 0;
  buf_n = 0;
  b2_hff_save.lag_counter = 0;
  b2_hff_state.lag_counter = GPS_LAG_N;
  lag_counter_err = 0;
#endif
}

static inline void b2_hff_init_x(float init_x, float init_xdot, float init_xbias) {
  b2_hff_state.x     = init_x;
  b2_hff_state.xdot  = init_xdot;
  b2_hff_state.xbias = init_xbias;
  int i, j;
  for (i=0; i<B2_HFF_STATE_SIZE; i++) {
    for (j=0; j<B2_HFF_STATE_SIZE; j++)
      b2_hff_state.xP[i][j] = 0.;
	if (i < 2)
	  b2_hff_state.xP[i][i] = INIT_PXX;
	else
	  b2_hff_state.xP[i][i] = INIT_PXX_BIAS;
  }

}

static inline void b2_hff_init_y(float init_y, float init_ydot, float init_ybias) {
  b2_hff_state.y     = init_y;
  b2_hff_state.ydot  = init_ydot;
  b2_hff_state.ybias = init_ybias;
  int i, j;
  for (i=0; i<B2_HFF_STATE_SIZE; i++) {
    for (j=0; j<B2_HFF_STATE_SIZE; j++)
      b2_hff_state.yP[i][j] = 0.;
	if (i < 2)
	  b2_hff_state.yP[i][i] = INIT_PXX;
	else
	  b2_hff_state.yP[i][i] = INIT_PXX_BIAS;
  }
}

#ifdef GPS_LAG
void b2_hff_store_accel(float x, float y) {
  past_accel[buf_w].x = x;
  past_accel[buf_w].y = y;
  buf_w = (buf_w + 1) < BUF_MAXN ? (buf_w + 1) : 0;

  if (buf_n < BUF_MAXN) {
	buf_n++;
  } else {
	buf_r = (buf_r + 1) < BUF_MAXN ? (buf_r + 1) : 0;
  }
}

/* get the accel values from back_n steps ago */
static inline void b2_hff_get_past_accel(int back_n) {
  int i;
  if (back_n > buf_n) {
	//printf("Cannot go back %d steps, going back only %d instead!\n", back_n, buf_n);
	back_n = buf_n;
  }
  //i = (buf_r + n) < BUF_MAXN ? (buf_r + n) : (buf_r + n - BUF_MAXN);
  i = (buf_w-1 - back_n) > 0 ? (buf_w-1 - back_n) : (buf_w-1 + BUF_MAXN - back_n);
  b2_hff_xaccel = past_accel[i].x;
  b2_hff_yaccel = past_accel[i].y;
}

/* rollback the state and covariance matrix to time when last saved */
static inline void b2_hff_rollback_filter(void) {
  /* b2_hff_x = b2_hff_x_sav; */
/*   b2_hff_xbias = b2_hff_xbias_sav; */
/*   b2_hff_xdot = b2_hff_xdot_sav; */
/*   b2_hff_xdotdot = b2_hff_xdotdot_sav; */
/*   b2_hff_y = b2_hff_y_sav; */
/*   b2_hff_ybias = b2_hff_ybias_sav; */
/*   b2_hff_ydot = b2_hff_ydot_sav; */
/*   b2_hff_ydotdot = b2_hff_ydotdot_sav; */
/*   for (int i=0; i < B2_HFF_STATE_SIZE; i++) { */
/* 	for (int j=0; j < B2_HFF_STATE_SIZE; j++) { */
/* 	  b2_hff_xP[i][j] = b2_hff_xP_sav[i][j]; */
/* 	  b2_hff_yP[i][j] = b2_hff_yP_sav[i][j]; */
/* 	} */
/*   } */
  b2_hff_work = &b2_hff_save;
}

/* save current state for later rollback */
static inline void b2_hff_save_filter(void) {
  b2_hff_save.x       = b2_hff_state.x;
  b2_hff_save.xbias   = b2_hff_state.xbias;
  b2_hff_save.xdot    = b2_hff_state.xdot;
  b2_hff_save.xdotdot = b2_hff_state.xdotdot;
  b2_hff_save.y       = b2_hff_state.y;
  b2_hff_save.ybias   = b2_hff_state.ybias;
  b2_hff_save.ydot    = b2_hff_state.ydot;
  b2_hff_save.ydotdot = b2_hff_state.ydotdot;
  for (int i=0; i < B2_HFF_STATE_SIZE; i++) {
	for (int j=0; j < B2_HFF_STATE_SIZE; j++) {
	  b2_hff_save.xP[i][j] = b2_hff_state.xP[i][j];
	  b2_hff_save.yP[i][j] = b2_hff_state.yP[i][j];
	}
  }
  b2_hff_save.lag_counter = 0;
}

/* copy working state to output state */
static inline void b2_hff_set_state(struct hfilter_f* source) {
  b2_hff_state.x       = source->x;
  b2_hff_state.xbias   = source->xbias;
  b2_hff_state.xdot    = source->xdot;
  b2_hff_state.xdotdot = source->xdotdot;
  b2_hff_state.y       = source->y;
  b2_hff_state.ybias   = source->ybias;
  b2_hff_state.ydot    = source->ydot;
  b2_hff_state.ydotdot = source->ydotdot;
  for (int i=0; i < B2_HFF_STATE_SIZE; i++) {
	for (int j=0; j < B2_HFF_STATE_SIZE; j++) {
	  b2_hff_state.xP[i][j] = source->xP[i][j];
	  b2_hff_state.yP[i][j] = source->yP[i][j];
	}
  }
}
#endif

/*

 F = [ 1 dt -dt^2/2
       0  1 -dt
       0  0   1     ];

 B = [ dt^2/2 dt 0]';

 Q = [ 0.01  0     0
       0     0.01  0
       0     0     0.001 ];

 Xk1 = F * Xk0 + B * accel;

 Pk1 = F * Pk0 * F' + Q;

*/
void b2_hff_propagate(float xaccel, float yaccel) {
#ifdef GPS_LAG
  /* save filter if now is the estimated GPS validity point in time */
  if (save_counter == 0) {
	b2_hff_save_filter();
	save_counter = -1;
  } else if (save_counter > 0) {
	save_counter--;
  }
#endif

  b2_hff_xaccel = xaccel;
  b2_hff_yaccel = yaccel;
  b2_hff_propagate_x();
  b2_hff_propagate_y();

#ifdef GPS_LAG
  if (b2_hff_save.lag_counter < 2*GPS_LAG_N) // prevent from overflowing if no gps available
	b2_hff_save.lag_counter++;
#endif
}

static inline void b2_hff_propagate_x(void) {
  /* update state */
  b2_hff_work->xdotdot = b2_hff_xaccel - b2_hff_work->xbias;
  b2_hff_work->x = b2_hff_work->x + DT_HFILTER * b2_hff_work->xdot + DT_HFILTER * DT_HFILTER / 2 * b2_hff_work->xdotdot;
  b2_hff_work->xdot = b2_hff_work->xdot + DT_HFILTER * b2_hff_work->xdotdot;
  /* update covariance */
  const float FPF00 = b2_hff_work->xP[0][0] + DT_HFILTER * ( b2_hff_work->xP[1][0] + b2_hff_work->xP[0][1] + DT_HFILTER * b2_hff_work->xP[1][1] );
  const float FPF01 = b2_hff_work->xP[0][1] + DT_HFILTER * ( b2_hff_work->xP[1][1] - b2_hff_work->xP[0][2] - DT_HFILTER * b2_hff_work->xP[1][2] );
  const float FPF02 = b2_hff_work->xP[0][2] + DT_HFILTER * ( b2_hff_work->xP[1][2] );
  const float FPF10 = b2_hff_work->xP[1][0] + DT_HFILTER * (-b2_hff_work->xP[2][0] + b2_hff_work->xP[1][1] - DT_HFILTER * b2_hff_work->xP[2][1] );
  const float FPF11 = b2_hff_work->xP[1][1] + DT_HFILTER * (-b2_hff_work->xP[2][1] - b2_hff_work->xP[1][2] + DT_HFILTER * b2_hff_work->xP[2][2] );
  const float FPF12 = b2_hff_work->xP[1][2] + DT_HFILTER * (-b2_hff_work->xP[2][2] );
  const float FPF20 = b2_hff_work->xP[2][0] + DT_HFILTER * ( b2_hff_work->xP[2][1] );
  const float FPF21 = b2_hff_work->xP[2][1] + DT_HFILTER * (-b2_hff_work->xP[2][2] );
  const float FPF22 = b2_hff_work->xP[2][2];

  b2_hff_work->xP[0][0] = FPF00 + Q;
  b2_hff_work->xP[0][1] = FPF01;
  b2_hff_work->xP[0][2] = FPF02;
  b2_hff_work->xP[1][0] = FPF10;
  b2_hff_work->xP[1][1] = FPF11 + Qdotdot;
  b2_hff_work->xP[1][2] = FPF12;
  b2_hff_work->xP[2][0] = FPF20;
  b2_hff_work->xP[2][1] = FPF21;
  b2_hff_work->xP[2][2] = FPF22 + Qbiasbias;

}

static inline void b2_hff_propagate_y(void) {
  /* update state */
  b2_hff_work->ydotdot = b2_hff_yaccel - b2_hff_work->ybias;
  b2_hff_work->y = b2_hff_work->y + DT_HFILTER * b2_hff_work->ydot;
  b2_hff_work->ydot = b2_hff_work->ydot + DT_HFILTER * b2_hff_work->ydotdot;
  /* update covariance */
  const float FPF00 = b2_hff_work->yP[0][0] + DT_HFILTER * ( b2_hff_work->yP[1][0] + b2_hff_work->yP[0][1] + DT_HFILTER * b2_hff_work->yP[1][1] );
  const float FPF01 = b2_hff_work->yP[0][1] + DT_HFILTER * ( b2_hff_work->yP[1][1] - b2_hff_work->yP[0][2] - DT_HFILTER * b2_hff_work->yP[1][2] );
  const float FPF02 = b2_hff_work->yP[0][2] + DT_HFILTER * ( b2_hff_work->yP[1][2] );
  const float FPF10 = b2_hff_work->yP[1][0] + DT_HFILTER * (-b2_hff_work->yP[2][0] + b2_hff_work->yP[1][1] - DT_HFILTER * b2_hff_work->yP[2][1] );
  const float FPF11 = b2_hff_work->yP[1][1] + DT_HFILTER * (-b2_hff_work->yP[2][1] - b2_hff_work->yP[1][2] + DT_HFILTER * b2_hff_work->yP[2][2] );
  const float FPF12 = b2_hff_work->yP[1][2] + DT_HFILTER * (-b2_hff_work->yP[2][2] );
  const float FPF20 = b2_hff_work->yP[2][0] + DT_HFILTER * ( b2_hff_work->yP[2][1] );
  const float FPF21 = b2_hff_work->yP[2][1] + DT_HFILTER * (-b2_hff_work->yP[2][2] );
  const float FPF22 = b2_hff_work->yP[2][2];

  b2_hff_work->yP[0][0] = FPF00 + Q;
  b2_hff_work->yP[0][1] = FPF01;
  b2_hff_work->yP[0][2] = FPF02;
  b2_hff_work->yP[1][0] = FPF10;
  b2_hff_work->yP[1][1] = FPF11 + Qdotdot;
  b2_hff_work->yP[1][2] = FPF12;
  b2_hff_work->yP[2][0] = FPF20;
  b2_hff_work->yP[2][1] = FPF21;
  b2_hff_work->yP[2][2] = FPF22 + Qbiasbias;

}


void b2_hff_update_gps(void) {
#ifdef GPS_LAG
  lag_counter_err = b2_hff_save.lag_counter - GPS_LAG_N;
  /* roll back if state was saved approx when GPS was valid */
  if (abs(lag_counter_err) < 3) {
	b2_hff_rollback_filter();
  }
#endif

#ifdef B2_HFF_UPDATE_POS
  b2_hff_update_x(booz_ins_gps_pos_m_ned.x);
  b2_hff_update_y(booz_ins_gps_pos_m_ned.y);
#endif
#ifdef B2_HFF_UPDATE_SPEED
  b2_hff_update_xdot(booz_ins_gps_speed_m_s_ned.x);
  b2_hff_update_ydot(booz_ins_gps_speed_m_s_ned.y);
#endif

#ifdef GPS_LAG
  /* roll back if state was saved approx when GPS was valid */
  if (abs(lag_counter_err) < 3) {
	/* redo all propagation steps since GPS update */
	for (int i=b2_hff_save.lag_counter-1; i >= 0; i--) {
	  b2_hff_get_past_accel(i);
	  b2_hff_propagate_x();
	  b2_hff_propagate_y();
	}
	b2_hff_set_state(b2_hff_work);
	b2_hff_work = &b2_hff_state;
  }

  /* reset save counter */
  save_counter = (int)(HFF_FREQ / 4) % GPS_LAG_N;
#endif
}


/*
  H = [1 0 0];
  R = 0.1;
  // state residual
  y = pos_measurement - H * Xm;
  // covariance residual
  S = H*Pm*H' + R;
  // kalman gain
  K = Pm*H'*inv(S);
  // update state
  Xp = Xm + K*y;
  // update covariance
  Pp = Pm - K*H*Pm;
*/
void b2_hff_update_pos (float posx, float posy) {
    b2_hff_update_x(posx);
    b2_hff_update_y(posy);
}

static inline void b2_hff_update_x(float x_meas) {
  b2_hff_x_meas = x_meas;

  const float y = x_meas - b2_hff_work->x;
  const float S  = b2_hff_work->xP[0][0] + Rpos;
  const float K1 = b2_hff_work->xP[0][0] * 1/S;
  const float K2 = b2_hff_work->xP[1][0] * 1/S;
  const float K3 = b2_hff_work->xP[2][0] * 1/S;

  b2_hff_work->x     = b2_hff_work->x     + K1 * y;
  b2_hff_work->xdot  = b2_hff_work->xdot  + K2 * y;
  b2_hff_work->xbias = b2_hff_work->xbias + K3 * y;

  const float P11 = (1. - K1) * b2_hff_work->xP[0][0];
  const float P12 = (1. - K1) * b2_hff_work->xP[0][1];
  const float P13 = (1. - K1) * b2_hff_work->xP[0][2];
  const float P21 = -K2 * b2_hff_work->xP[0][0] + b2_hff_work->xP[1][0];
  const float P22 = -K2 * b2_hff_work->xP[0][1] + b2_hff_work->xP[1][1];
  const float P23 = -K2 * b2_hff_work->xP[0][2] + b2_hff_work->xP[1][2];
  const float P31 = -K3 * b2_hff_work->xP[0][0] + b2_hff_work->xP[2][0];
  const float P32 = -K3 * b2_hff_work->xP[0][1] + b2_hff_work->xP[2][1];
  const float P33 = -K3 * b2_hff_work->xP[0][2] + b2_hff_work->xP[2][2];

  b2_hff_work->xP[0][0] = P11;
  b2_hff_work->xP[0][1] = P12;
  b2_hff_work->xP[0][2] = P13;
  b2_hff_work->xP[1][0] = P21;
  b2_hff_work->xP[1][1] = P22;
  b2_hff_work->xP[1][2] = P23;
  b2_hff_work->xP[2][0] = P31;
  b2_hff_work->xP[2][1] = P32;
  b2_hff_work->xP[2][2] = P33;

}

static inline void b2_hff_update_y(float y_meas) {
  b2_hff_y_meas = y_meas;

  const float y = y_meas - b2_hff_work->y;
  const float S  = b2_hff_work->yP[0][0] + Rpos;
  const float K1 = b2_hff_work->yP[0][0] * 1/S;
  const float K2 = b2_hff_work->yP[1][0] * 1/S;
  const float K3 = b2_hff_work->yP[2][0] * 1/S;

  b2_hff_work->y     = b2_hff_work->y     + K1 * y;
  b2_hff_work->ydot  = b2_hff_work->ydot  + K2 * y;
  b2_hff_work->ybias = b2_hff_work->ybias + K3 * y;

  const float P11 = (1. - K1) * b2_hff_work->yP[0][0];
  const float P12 = (1. - K1) * b2_hff_work->yP[0][1];
  const float P13 = (1. - K1) * b2_hff_work->yP[0][2];
  const float P21 = -K2 * b2_hff_work->yP[0][0] + b2_hff_work->yP[1][0];
  const float P22 = -K2 * b2_hff_work->yP[0][1] + b2_hff_work->yP[1][1];
  const float P23 = -K2 * b2_hff_work->yP[0][2] + b2_hff_work->yP[1][2];
  const float P31 = -K3 * b2_hff_work->yP[0][0] + b2_hff_work->yP[2][0];
  const float P32 = -K3 * b2_hff_work->yP[0][1] + b2_hff_work->yP[2][1];
  const float P33 = -K3 * b2_hff_work->yP[0][2] + b2_hff_work->yP[2][2];

  b2_hff_work->yP[0][0] = P11;
  b2_hff_work->yP[0][1] = P12;
  b2_hff_work->yP[0][2] = P13;
  b2_hff_work->yP[1][0] = P21;
  b2_hff_work->yP[1][1] = P22;
  b2_hff_work->yP[1][2] = P23;
  b2_hff_work->yP[2][0] = P31;
  b2_hff_work->yP[2][1] = P32;
  b2_hff_work->yP[2][2] = P33;

}




/*
  H = [0 1 0];
  R = 0.1;
  // state residual
  yd = vx - H * Xm;
  // covariance residual
  S = H*Pm*H' + R;
  // kalman gain
  K = Pm*H'*inv(S);
  // update state
  Xp = Xm + K*yd;
  // update covariance
  Pp = Pm - K*H*Pm;
*/
void b2_hff_update_v(float xspeed, float yspeed) {
    b2_hff_update_xdot(xspeed);
    b2_hff_update_ydot(yspeed);
}

static inline void b2_hff_update_xdot(float v) {
  const float yd = v - b2_hff_work->xdot;
  const float S  = b2_hff_work->xP[1][1] + Rspeed;
  const float K1 = b2_hff_work->xP[0][1] * 1/S;
  const float K2 = b2_hff_work->xP[1][1] * 1/S;
  const float K3 = b2_hff_work->xP[2][1] * 1/S;

  b2_hff_work->x     = b2_hff_work->x     + K1 * yd;
  b2_hff_work->xdot  = b2_hff_work->xdot  + K2 * yd;
  b2_hff_work->xbias = b2_hff_work->xbias + K3 * yd;

  const float P11 = -K1 * b2_hff_work->xP[1][0] + b2_hff_work->xP[0][0];
  const float P12 = -K1 * b2_hff_work->xP[1][1] + b2_hff_work->xP[0][1];
  const float P13 = -K1 * b2_hff_work->xP[1][2] + b2_hff_work->xP[0][2];
  const float P21 = (1. - K2) * b2_hff_work->xP[1][0];
  const float P22 = (1. - K2) * b2_hff_work->xP[1][1];
  const float P23 = (1. - K2) * b2_hff_work->xP[1][2];
  const float P31 = -K3 * b2_hff_work->xP[1][0] + b2_hff_work->xP[2][0];
  const float P32 = -K3 * b2_hff_work->xP[1][1] + b2_hff_work->xP[2][1];
  const float P33 = -K3 * b2_hff_work->xP[1][2] + b2_hff_work->xP[2][2];

  b2_hff_work->xP[0][0] = P11;
  b2_hff_work->xP[0][1] = P12;
  b2_hff_work->xP[0][2] = P13;
  b2_hff_work->xP[1][0] = P21;
  b2_hff_work->xP[1][1] = P22;
  b2_hff_work->xP[1][2] = P23;
  b2_hff_work->xP[2][0] = P31;
  b2_hff_work->xP[2][1] = P32;
  b2_hff_work->xP[2][2] = P33;

}

static inline void b2_hff_update_ydot(float v) {
  const float yd = v - b2_hff_work->ydot;
  const float S  = b2_hff_work->yP[1][1] + Rspeed;
  const float K1 = b2_hff_work->yP[0][1] * 1/S;
  const float K2 = b2_hff_work->yP[1][1] * 1/S;
  const float K3 = b2_hff_work->yP[2][1] * 1/S;

  b2_hff_work->y     = b2_hff_work->y     + K1 * yd;
  b2_hff_work->ydot  = b2_hff_work->ydot  + K2 * yd;
  b2_hff_work->ybias = b2_hff_work->ybias + K3 * yd;

  const float P11 = -K1 * b2_hff_work->yP[1][0] + b2_hff_work->yP[0][0];
  const float P12 = -K1 * b2_hff_work->yP[1][1] + b2_hff_work->yP[0][1];
  const float P13 = -K1 * b2_hff_work->yP[1][2] + b2_hff_work->yP[0][2];
  const float P21 = (1. - K2) * b2_hff_work->yP[1][0];
  const float P22 = (1. - K2) * b2_hff_work->yP[1][1];
  const float P23 = (1. - K2) * b2_hff_work->yP[1][2];
  const float P31 = -K3 * b2_hff_work->yP[1][0] + b2_hff_work->yP[2][0];
  const float P32 = -K3 * b2_hff_work->yP[1][1] + b2_hff_work->yP[2][1];
  const float P33 = -K3 * b2_hff_work->yP[1][2] + b2_hff_work->yP[2][2];

  b2_hff_work->yP[0][0] = P11;
  b2_hff_work->yP[0][1] = P12;
  b2_hff_work->yP[0][2] = P13;
  b2_hff_work->yP[1][0] = P21;
  b2_hff_work->yP[1][1] = P22;
  b2_hff_work->yP[1][2] = P23;
  b2_hff_work->yP[2][0] = P31;
  b2_hff_work->yP[2][1] = P32;
  b2_hff_work->yP[2][2] = P33;

}


