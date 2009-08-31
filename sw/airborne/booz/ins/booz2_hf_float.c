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
#include "booz_imu.h"
#include "booz_ahrs.h"
#include <stdlib.h>



/* horizontal filter propagation frequency */
#define HFF_FREQ 512./HFF_PRESCALER
#define DT_HFILTER 1./HFF_FREQ

/* initial covariance diagonal */
#define INIT_PXX 1.
/* process noise (is the same for x and y)*/
#define ACCEL_NOISE 0.5
#define Q       ACCEL_NOISE*DT_HFILTER*DT_HFILTER/2.
#define Qdotdot ACCEL_NOISE*DT_HFILTER
//TODO: proper measurement noise
#define Rpos   5.
#define Rspeed 1.

/*

  X_x = [ x xdot]
  X_y = [ y ydot]


*/
/* output filter states */
struct HfilterFloat b2_hff_state;
/* pointer to filter states to operate on */
//struct HfilterFloat *b2_hff_work;



/* acceleration used in propagation step */
float b2_hff_xaccel;
float b2_hff_yaccel;

/* last position measurement */
float b2_hff_x_meas;
float b2_hff_y_meas;

/* counter for hff propagation*/
int b2_hff_ps_counter;


/*
 * For GPS lag compensation
 *
 *
 *
 */

#ifdef GPS_LAG
/*
 * GPS_LAG is defined in seconds in airframe file
 */

/* number of propagaton steps to redo according to GPS_LAG */
#define GPS_LAG_N ((int) (GPS_LAG * HFF_FREQ + 0.5))
/* number of propagation steps between two GPS updates */
#define GPS_DT_N ((int) (HFF_FREQ / 4))

/* maximum number of past propagation steps to rerun per ap cycle
 * make sure GPS_LAG_N/MAX_PP_STEPS < GPS_DT_N
 */
#define MAX_PP_STEPS 6

/* variables for accel buffer */
#define ACC_BUF_MAXN GPS_LAG_N+10
struct FloatVect2 past_accel[ACC_BUF_MAXN]; /* buffer with past mean accel values for redoing the propagation */
uint8_t acc_buf_r; /* pos to read from, oldest measurement */
uint8_t acc_buf_w; /* pos to write to */
uint8_t acc_buf_n; /* number of elements in buffer */


/*
 * stuff for ringbuffer to store past filter states
 */
#define HFF_RB_MAXN ((int) (GPS_LAG * 4))
#define INC_RB_POINTER(ptr) {					\
	if (ptr == &b2_hff_rb[HFF_RB_MAXN-1])		\
	ptr = b2_hff_rb;							\
  else											\
	ptr++;										\
  }

struct HfilterFloat b2_hff_rb[HFF_RB_MAXN]; /* ringbuffer with state and covariance when GPS was valid */
struct HfilterFloat *b2_hff_rb_put; /* write pointer */
#endif /* GPS_LAG */

struct HfilterFloat *b2_hff_rb_last; /* read pointer */
int b2_hff_rb_n; /* fill count */


/* by how many steps the estimated GPS validity point in time differed from GPS_LAG_N */
int8_t lag_counter_err;

/* counts down the propagation steps until the filter state is saved again */
int8_t save_counter;
int8_t past_save_counter;

#ifdef GPS_LAG
static inline void b2_hff_get_past_accel(int back_n);
static inline void b2_hff_rb_put_state(struct HfilterFloat* source);
static inline void b2_hff_rb_next(void);
static inline void b2_hff_set_state(struct HfilterFloat* dest, struct HfilterFloat* source);
#endif



static inline void b2_hff_init_x(float init_x, float init_xdot);
static inline void b2_hff_init_y(float init_y, float init_ydot);

static inline void b2_hff_propagate_x(struct HfilterFloat* hff_work);
static inline void b2_hff_propagate_y(struct HfilterFloat* hff_work);

static inline void b2_hff_update_x(struct HfilterFloat* hff_work, float x_meas);
static inline void b2_hff_update_y(struct HfilterFloat* hff_work, float y_meas);

static inline void b2_hff_update_xdot(struct HfilterFloat* hff_work, float v);
static inline void b2_hff_update_ydot(struct HfilterFloat* hff_work, float v);



void b2_hff_init(float init_x, float init_xdot, float init_y, float init_ydot) {
  b2_hff_init_x(init_x, init_xdot);
  b2_hff_init_y(init_y, init_ydot);
#ifdef GPS_LAG
  acc_buf_r = 0;
  acc_buf_w = 0;
  acc_buf_n = 0;
  b2_hff_rb_put = b2_hff_rb;
  b2_hff_rb_last = b2_hff_rb;
  b2_hff_state.lag_counter = GPS_LAG_N;
#else
  b2_hff_rb_last = NULL;
  b2_hff_state.lag_counter = 0;
#endif
  b2_hff_rb_n = 0;
  b2_hff_state.rollback = 0;
  lag_counter_err = 0;
  b2_hff_ps_counter = 1;
}

static inline void b2_hff_init_x(float init_x, float init_xdot) {
  b2_hff_state.x     = init_x;
  b2_hff_state.xdot  = init_xdot;
  int i, j;
  for (i=0; i<B2_HFF_STATE_SIZE; i++) {
    for (j=0; j<B2_HFF_STATE_SIZE; j++)
      b2_hff_state.xP[i][j] = 0.;
	b2_hff_state.xP[i][i] = INIT_PXX;
  }

}

static inline void b2_hff_init_y(float init_y, float init_ydot) {
  b2_hff_state.y     = init_y;
  b2_hff_state.ydot  = init_ydot;
  int i, j;
  for (i=0; i<B2_HFF_STATE_SIZE; i++) {
    for (j=0; j<B2_HFF_STATE_SIZE; j++)
      b2_hff_state.yP[i][j] = 0.;
	b2_hff_state.yP[i][i] = INIT_PXX;
  }
}

#ifdef GPS_LAG
void b2_hff_store_accel(float x, float y) {
  past_accel[acc_buf_w].x = x;
  past_accel[acc_buf_w].y = y;
  acc_buf_w = (acc_buf_w + 1) < ACC_BUF_MAXN ? (acc_buf_w + 1) : 0;

  if (acc_buf_n < ACC_BUF_MAXN) {
	acc_buf_n++;
  } else {
	acc_buf_r = (acc_buf_r + 1) < ACC_BUF_MAXN ? (acc_buf_r + 1) : 0;
  }
}

/* get the accel values from back_n steps ago */
static inline void b2_hff_get_past_accel(int back_n) {
  int i;
  if (back_n > acc_buf_n) {
	//printf("Cannot go back %d steps, going back only %d instead!\n", back_n, acc_buf_n);
	back_n = acc_buf_n;
  } else if (back_n <= 0) {
	//printf("Cannot go back %d steps, not getting past accel value!\n", back_n);
	return;
  }
  i = (acc_buf_w - back_n) > 0 ? (acc_buf_w - back_n) : (acc_buf_w + ACC_BUF_MAXN - back_n);
  b2_hff_xaccel = past_accel[i].x;
  b2_hff_yaccel = past_accel[i].y;
}

static inline void b2_hff_rb_put_state(struct HfilterFloat* source) {
  /* copy state from source into buffer */
  b2_hff_set_state(b2_hff_rb_put, source);
  b2_hff_rb_put->lag_counter = 0;
  b2_hff_rb_put->rollback = 0;

  /* forward write pointer */
  INC_RB_POINTER(b2_hff_rb_put);

  /* increase fill count and forward last pointer if neccessary */
  if (b2_hff_rb_n < HFF_RB_MAXN) {
	b2_hff_rb_n++;
  } else {
	INC_RB_POINTER(b2_hff_rb_last);
  }
}

static inline void b2_hff_rb_next(void) {
  if (b2_hff_rb_n > 0) {
	INC_RB_POINTER(b2_hff_rb_last);
	b2_hff_rb_n--;
  } else {
	//printf("hff ringbuffer empty!\n");
  }
}


/* copy source state to dest state */
static inline void b2_hff_set_state(struct HfilterFloat* dest, struct HfilterFloat* source) {
  dest->x       = source->x;
  dest->xdot    = source->xdot;
  dest->xdotdot = source->xdotdot;
  dest->y       = source->y;
  dest->ydot    = source->ydot;
  dest->ydotdot = source->ydotdot;
  for (int i=0; i < B2_HFF_STATE_SIZE; i++) {
	for (int j=0; j < B2_HFF_STATE_SIZE; j++) {
	  dest->xP[i][j] = source->xP[i][j];
	  dest->yP[i][j] = source->yP[i][j];
	}
  }
}

static inline void b2_hff_propagate_past(struct HfilterFloat* hff_past) {
  /* run max MAX_PP_STEPS propagation steps */
  for (int i=0; i < MAX_PP_STEPS; i++) {
	b2_hff_get_past_accel(hff_past->lag_counter);
	b2_hff_propagate_x(hff_past);
	b2_hff_propagate_y(hff_past);
	hff_past->lag_counter--;

	if (past_save_counter == 0) {
	  /* next GPS measurement valid at this state -> save */
	  b2_hff_rb_put_state(hff_past);
	  past_save_counter = -1;
	} else if (save_counter > 0) {
	  past_save_counter--;
	}

	if (hff_past->lag_counter == 0) {
	  b2_hff_set_state(&b2_hff_state, hff_past);
	  b2_hff_rb_next();
	  break;
	}
  }
}
#endif



void b2_hff_propagate(void) {
#ifdef GPS_LAG
  /* continue to redo the propagation to catch up with the present */
  if (b2_hff_rb_last->rollback) {
	b2_hff_propagate_past(b2_hff_rb_last);
  }
#endif

  /* propagate current state if it is time */
  if (b2_hff_ps_counter == HFF_PRESCALER) {
	b2_hff_ps_counter = 1;

	/* compute float ltp mean acceleration */
	booz_ahrs_compute_accel_mean(HFF_PRESCALER);
	struct Int32Vect3 mean_accel_body;
	INT32_RMAT_TRANSP_VMULT(mean_accel_body, booz_imu.body_to_imu_rmat, booz_ahrs_accel_mean);
	struct Int32Vect3 mean_accel_ltp;
	INT32_RMAT_TRANSP_VMULT(mean_accel_ltp, booz_ahrs.ltp_to_body_rmat, mean_accel_body);
	b2_hff_xaccel = ACCEL_FLOAT_OF_BFP(mean_accel_ltp.x);
	b2_hff_yaccel = ACCEL_FLOAT_OF_BFP(mean_accel_ltp.y);

	/*
	 * propagate current state
	 */
	if ( booz_ins_ltp_initialised ) {
	  b2_hff_propagate_x(&b2_hff_state);
	  b2_hff_propagate_y(&b2_hff_state);
	}
#ifdef GPS_LAG
	b2_hff_store_accel(b2_hff_xaccel, b2_hff_yaccel);
	/* increase all lag counters */
	for (int i=0; i < b2_hff_rb_n; i++ ) {
	  (b2_hff_rb_last+i)->lag_counter++;
	}

	/* save filter state if ringbuffer is empty */
	if (save_counter == 0) {
	  b2_hff_rb_put_state(&b2_hff_state);
	  save_counter = -1;
	} else if (save_counter > 0) {
	  save_counter--;
	}
#endif

  } else {
	b2_hff_ps_counter++;
  }
}




void b2_hff_update_gps(void) {
#ifdef GPS_LAG

  if (b2_hff_rb_n > 0) {
	/* roll back if state was saved approx when GPS was valid */
	lag_counter_err = b2_hff_rb_last->lag_counter - GPS_LAG_N;
	if (abs(lag_counter_err) < 3) {
	  b2_hff_rb_last->rollback = 1;
	  b2_hff_update_x(b2_hff_rb_last, booz_ins_gps_pos_m_ned.x);
	  b2_hff_update_y(b2_hff_rb_last, booz_ins_gps_pos_m_ned.y);
#ifdef B2_HFF_UPDATE_SPEED
	  b2_hff_update_xdot(b2_hff_rb_last, booz_ins_gps_speed_m_s_ned.x);
	  b2_hff_update_ydot(b2_hff_rb_last, booz_ins_gps_speed_m_s_ned.y);
#endif
	  past_save_counter = GPS_DT_N + lag_counter_err;
	  b2_hff_propagate_past(b2_hff_rb_last);
	} else if (lag_counter_err >= GPS_DT_N-2) {
	  /* apparently missed a GPS update, try next saved state */
	  b2_hff_rb_next();
	  b2_hff_update_gps();
	}
  } else {
	/* ringbuffer empty -> save output filter state at next GPS validity point in time */
	save_counter = GPS_DT_N - (GPS_LAG_N % GPS_DT_N);
  }

#else /* GPS_LAG */

  b2_hff_update_x(&b2_hff_state, booz_ins_gps_pos_m_ned.x);
  b2_hff_update_y(&b2_hff_state, booz_ins_gps_pos_m_ned.y);
#ifdef B2_HFF_UPDATE_SPEED
  b2_hff_update_xdot(&b2_hff_state, booz_ins_gps_speed_m_s_ned.x);
  b2_hff_update_ydot(&b2_hff_state, booz_ins_gps_speed_m_s_ned.y);
#endif

#endif/* GPS_LAG */
}



/*
 *
 * Propagation
 *
 *

  F = [ 1 dt
        0  1 ];

  B = [ dt^2/2 dt]';

  Q = [ 0.01  0
        0     0.01];

  Xk1 = F * Xk0 + B * accel;

  Pk1 = F * Pk0 * F' + Q;

*/
static inline void b2_hff_propagate_x(struct HfilterFloat* hff_work) {
  /* update state */
  hff_work->xdotdot = b2_hff_xaccel;
  hff_work->x = hff_work->x + DT_HFILTER * hff_work->xdot;
  hff_work->xdot = hff_work->xdot + DT_HFILTER * hff_work->xdotdot;
  /* update covariance */
  const float FPF00 = hff_work->xP[0][0] + DT_HFILTER * ( hff_work->xP[1][0] + hff_work->xP[0][1] + DT_HFILTER * hff_work->xP[1][1] );
  const float FPF01 = hff_work->xP[0][1] + DT_HFILTER * hff_work->xP[1][1];
  const float FPF10 = hff_work->xP[1][0] + DT_HFILTER * hff_work->xP[1][1];
  const float FPF11 = hff_work->xP[1][1];

  hff_work->xP[0][0] = FPF00 + Q;
  hff_work->xP[0][1] = FPF01;
  hff_work->xP[1][0] = FPF10;
  hff_work->xP[1][1] = FPF11 + Qdotdot;
}

static inline void b2_hff_propagate_y(struct HfilterFloat* hff_work) {
  /* update state */
  hff_work->ydotdot = b2_hff_yaccel;
  hff_work->y = hff_work->y + DT_HFILTER * hff_work->ydot;
  hff_work->ydot = hff_work->ydot + DT_HFILTER * hff_work->ydotdot;
  /* update covariance */
  const float FPF00 = hff_work->yP[0][0] + DT_HFILTER * ( hff_work->yP[1][0] + hff_work->yP[0][1] + DT_HFILTER * hff_work->yP[1][1] );
  const float FPF01 = hff_work->yP[0][1] + DT_HFILTER * hff_work->yP[1][1];
  const float FPF10 = hff_work->yP[1][0] + DT_HFILTER * hff_work->yP[1][1];
  const float FPF11 = hff_work->yP[1][1];

  hff_work->yP[0][0] = FPF00 + Q;
  hff_work->yP[0][1] = FPF01;
  hff_work->yP[1][0] = FPF10;
  hff_work->yP[1][1] = FPF11 + Qdotdot;
}


/*
 *
 * Update position
 *
 *

  H = [1 0];
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
  b2_hff_update_x(&b2_hff_state, posx);
  b2_hff_update_y(&b2_hff_state, posy);
}

static inline void b2_hff_update_x(struct HfilterFloat* hff_work, float x_meas) {
  b2_hff_x_meas = x_meas;

  const float y  = x_meas - hff_work->x;
  const float S  = hff_work->xP[0][0] + Rpos;
  const float K1 = hff_work->xP[0][0] * 1/S;
  const float K2 = hff_work->xP[1][0] * 1/S;

  hff_work->x     = hff_work->x     + K1 * y;
  hff_work->xdot  = hff_work->xdot  + K2 * y;

  const float P11 = (1. - K1) * hff_work->xP[0][0];
  const float P12 = (1. - K1) * hff_work->xP[0][1];
  const float P21 = -K2 * hff_work->xP[0][0] + hff_work->xP[1][0];
  const float P22 = -K2 * hff_work->xP[0][1] + hff_work->xP[1][1];

  hff_work->xP[0][0] = P11;
  hff_work->xP[0][1] = P12;
  hff_work->xP[1][0] = P21;
  hff_work->xP[1][1] = P22;
}

static inline void b2_hff_update_y(struct HfilterFloat* hff_work, float y_meas) {
  b2_hff_y_meas = y_meas;

  const float y  = y_meas - hff_work->y;
  const float S  = hff_work->yP[0][0] + Rpos;
  const float K1 = hff_work->yP[0][0] * 1/S;
  const float K2 = hff_work->yP[1][0] * 1/S;

  hff_work->y     = hff_work->y     + K1 * y;
  hff_work->ydot  = hff_work->ydot  + K2 * y;

  const float P11 = (1. - K1) * hff_work->yP[0][0];
  const float P12 = (1. - K1) * hff_work->yP[0][1];
  const float P21 = -K2 * hff_work->yP[0][0] + hff_work->yP[1][0];
  const float P22 = -K2 * hff_work->yP[0][1] + hff_work->yP[1][1];

  hff_work->yP[0][0] = P11;
  hff_work->yP[0][1] = P12;
  hff_work->yP[1][0] = P21;
  hff_work->yP[1][1] = P22;
}




/*
 *
 * Update speed
 *
 *

  H = [0 1];
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
  b2_hff_update_xdot(&b2_hff_state, xspeed);
  b2_hff_update_ydot(&b2_hff_state, yspeed);
}

static inline void b2_hff_update_xdot(struct HfilterFloat* hff_work, float v) {
  const float yd = v - hff_work->xdot;
  const float S  = hff_work->xP[1][1] + Rspeed;
  const float K1 = hff_work->xP[0][1] * 1/S;
  const float K2 = hff_work->xP[1][1] * 1/S;

  hff_work->x     = hff_work->x     + K1 * yd;
  hff_work->xdot  = hff_work->xdot  + K2 * yd;

  const float P11 = -K1 * hff_work->xP[1][0] + hff_work->xP[0][0];
  const float P12 = -K1 * hff_work->xP[1][1] + hff_work->xP[0][1];
  const float P21 = (1. - K2) * hff_work->xP[1][0];
  const float P22 = (1. - K2) * hff_work->xP[1][1];

  hff_work->xP[0][0] = P11;
  hff_work->xP[0][1] = P12;
  hff_work->xP[1][0] = P21;
  hff_work->xP[1][1] = P22;
}

static inline void b2_hff_update_ydot(struct HfilterFloat* hff_work, float v) {
  const float yd = v - hff_work->ydot;
  const float S  = hff_work->yP[1][1] + Rspeed;
  const float K1 = hff_work->yP[0][1] * 1/S;
  const float K2 = hff_work->yP[1][1] * 1/S;

  hff_work->y     = hff_work->y     + K1 * yd;
  hff_work->ydot  = hff_work->ydot  + K2 * yd;

  const float P11 = -K1 * hff_work->yP[1][0] + hff_work->yP[0][0];
  const float P12 = -K1 * hff_work->yP[1][1] + hff_work->yP[0][1];
  const float P21 = (1. - K2) * hff_work->yP[1][0];
  const float P22 = (1. - K2) * hff_work->yP[1][1];

  hff_work->yP[0][0] = P11;
  hff_work->yP[0][1] = P12;
  hff_work->yP[1][0] = P21;
  hff_work->yP[1][1] = P22;
}


