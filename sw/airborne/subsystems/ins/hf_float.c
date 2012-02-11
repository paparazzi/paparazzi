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

#include "subsystems/ins/hf_float.h"
#include "subsystems/ins.h"
#include "subsystems/imu.h"
#include "subsystems/ahrs.h"
#include "subsystems/gps.h"
#include <stdlib.h>

#include "generated/airframe.h"

#ifdef SITL
#include <stdio.h>
#define DBG_LEVEL 1
#define PRINT_DBG(_l, _p) {						\
	if (DBG_LEVEL >= _l)						\
	  printf _p;								\
}
#else
#define PRINT_DBG(_l, _p) {}
#endif


/* initial covariance diagonal */
#define INIT_PXX 1.
/* process noise (is the same for x and y)*/
#ifndef HFF_ACCEL_NOISE
#define HFF_ACCEL_NOISE 0.5
#endif
#define Q       HFF_ACCEL_NOISE*DT_HFILTER*DT_HFILTER/2.
#define Qdotdot HFF_ACCEL_NOISE*DT_HFILTER

//TODO: proper measurement noise
#ifndef HFF_R_POS
#define HFF_R_POS   8.
#endif
#ifndef HFF_R_POS_MIN
#define HFF_R_POS_MIN 3.
#endif

#ifndef HFF_R_SPEED
#define HFF_R_SPEED 2.
#endif
#ifndef HFF_R_SPEED_MIN
#define HFF_R_SPEED_MIN 1.
#endif

/* gps measurement noise */
float Rgps_pos, Rgps_vel;

/*

  X_x = [ x xdot]
  X_y = [ y ydot]


*/
/* output filter states */
struct HfilterFloat b2_hff_state;


/* last acceleration measurement */
float b2_hff_xdd_meas;
float b2_hff_ydd_meas;

/* last velocity measurement */
float b2_hff_xd_meas;
float b2_hff_yd_meas;

/* last position measurement */
float b2_hff_x_meas;
float b2_hff_y_meas;

/* counter for hff propagation*/
int b2_hff_ps_counter;


/*
 * accel(in body frame) buffer
 */
#define ACC_RB_MAXN 64
struct AccBuf {
  struct Int32Vect3 buf[ACC_RB_MAXN];
  int r; /* pos to read from, oldest measurement */
  int w; /* pos to write to */
  int n; /* number of elements in rb */
  int size;
};
struct AccBuf acc_body;
struct Int32Vect3 acc_body_mean;

void b2_hff_store_accel_body(void) {
  INT32_RMAT_TRANSP_VMULT(acc_body.buf[acc_body.w], imu.body_to_imu_rmat,  imu.accel);
  acc_body.w = (acc_body.w + 1) < acc_body.size ? (acc_body.w + 1) : 0;

  /* once the buffer is full it always has the last acc_body.size accel measurements */
  if (acc_body.n < acc_body.size) {
	acc_body.n++;
  } else {
	acc_body.r = (acc_body.r + 1) < acc_body.size ? (acc_body.r + 1) : 0;
  }
}

/* compute the mean of the last n accel measurements */
static inline void b2_hff_compute_accel_body_mean(uint8_t n) {
  struct Int32Vect3 sum;
  int i, j;

  INT_VECT3_ZERO(sum);

  if (n > 1) {
    if (n > acc_body.n) {
      n = acc_body.n;
    }
    for (i = 1; i <= n; i++) {
      j = (acc_body.w - i) > 0 ? (acc_body.w - i) : (acc_body.w - i + acc_body.size);
      VECT3_ADD(sum, acc_body.buf[j]);
    }
	VECT3_SDIV(acc_body_mean, sum, n);
  } else {
	VECT3_COPY(acc_body_mean, sum);
  }
}

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
/* tolerance of the GPS lag accuracy is +- GPS_LAG_TOLERANCE seconds */
#define GPS_LAG_TOLERANCE 0.08
#define GPS_LAG_TOL_N ((int) (GPS_LAG_TOLERANCE * HFF_FREQ + 0.5))

/* maximum number of past propagation steps to rerun per ap cycle
 * make sure GPS_LAG_N/MAX_PP_STEPS < 128
 * GPS_LAG_N/MAX_PP_STEPS/512 = seconds until re-propagation catches up with the present
 */
#define MAX_PP_STEPS 6

/* variables for mean accel buffer */
#define ACC_BUF_MAXN (GPS_LAG_N+10)
#define INC_ACC_IDX(idx) {	idx = (idx + 1) < ACC_BUF_MAXN ? (idx + 1) : 0;	}

struct FloatVect2 past_accel[ACC_BUF_MAXN]; /* buffer with past mean accel values for redoing the propagation */
unsigned int acc_buf_r; /* pos to read from, oldest measurement */
unsigned int acc_buf_w; /* pos to write to */
unsigned int acc_buf_n; /* number of elements in buffer */


/*
 * stuff for ringbuffer to store past filter states
 */
#define HFF_RB_MAXN ((int) (GPS_LAG * 4))
#define INC_RB_POINTER(ptr) {					\
	if (ptr == &b2_hff_rb[HFF_RB_MAXN-1])		\
	  ptr = b2_hff_rb;							\
	else										\
	  ptr++;									\
  }

struct HfilterFloat b2_hff_rb[HFF_RB_MAXN]; /* ringbuffer with state and covariance when GPS was valid */
struct HfilterFloat *b2_hff_rb_put; /* write pointer */
#endif /* GPS_LAG */

struct HfilterFloat *b2_hff_rb_last; /* read pointer */
int b2_hff_rb_n; /* fill count */


/* by how many steps the estimated GPS validity point in time differed from GPS_LAG_N */
int lag_counter_err;

/* counts down the propagation steps until the filter state is saved again */
int save_counter;
int past_save_counter;
#define SAVE_NOW 0
#define SAVING -1
#define SAVE_DONE -2

uint16_t b2_hff_lost_limit;
uint16_t b2_hff_lost_counter;

#ifdef GPS_LAG
static inline void b2_hff_get_past_accel(unsigned int back_n);
static inline void b2_hff_rb_put_state(struct HfilterFloat* source);
static inline void b2_hff_rb_drop_last(void);
static inline void b2_hff_set_state(struct HfilterFloat* dest, struct HfilterFloat* source);
#endif



static inline void b2_hff_init_x(float init_x, float init_xdot);
static inline void b2_hff_init_y(float init_y, float init_ydot);

static inline void b2_hff_propagate_x(struct HfilterFloat* hff_work);
static inline void b2_hff_propagate_y(struct HfilterFloat* hff_work);

static inline void b2_hff_update_x(struct HfilterFloat* hff_work, float x_meas, float Rpos);
static inline void b2_hff_update_y(struct HfilterFloat* hff_work, float y_meas, float Rpos);

static inline void b2_hff_update_xdot(struct HfilterFloat* hff_work, float vel, float Rvel);
static inline void b2_hff_update_ydot(struct HfilterFloat* hff_work, float vel, float Rvel);



void b2_hff_init(float init_x, float init_xdot, float init_y, float init_ydot) {
  Rgps_pos = HFF_R_POS;
  Rgps_vel = HFF_R_SPEED;
  b2_hff_init_x(init_x, init_xdot);
  b2_hff_init_y(init_y, init_ydot);
  /* init buffer for mean accel calculation */
  acc_body.r = 0;
  acc_body.w = 0;
  acc_body.n = 0;
  acc_body.size = ACC_RB_MAXN;
#ifdef GPS_LAG
  /* init buffer for past mean accel values */
  acc_buf_r = 0;
  acc_buf_w = 0;
  acc_buf_n = 0;
  b2_hff_rb_put = b2_hff_rb;
  b2_hff_rb_last = b2_hff_rb;
  b2_hff_rb_last->rollback = FALSE;
  b2_hff_rb_last->lag_counter = 0;
  b2_hff_state.lag_counter = GPS_LAG_N;
#ifdef SITL
  printf("GPS_LAG: %f\n", GPS_LAG);
  printf("GPS_LAG_N: %d\n", GPS_LAG_N);
  printf("GPS_DT_N: %d\n", GPS_DT_N);
  printf("DT_HFILTER: %f\n", DT_HFILTER);
  printf("GPS_LAG_TOL_N: %i\n", GPS_LAG_TOL_N);
#endif
#else
  b2_hff_rb_last = &b2_hff_state;
  b2_hff_state.lag_counter = 0;
#endif
  b2_hff_rb_n = 0;
  b2_hff_state.rollback = FALSE;
  lag_counter_err = 0;
  save_counter = -1;
  past_save_counter = SAVE_DONE;
  b2_hff_ps_counter = 1;
  b2_hff_lost_counter = 0;
  b2_hff_lost_limit = HFF_LOST_LIMIT;
}

static inline void b2_hff_init_x(float init_x, float init_xdot) {
  b2_hff_state.x     = init_x;
  b2_hff_state.xdot  = init_xdot;
  int i, j;
  for (i=0; i<HFF_STATE_SIZE; i++) {
    for (j=0; j<HFF_STATE_SIZE; j++)
      b2_hff_state.xP[i][j] = 0.;
	b2_hff_state.xP[i][i] = INIT_PXX;
  }

}

static inline void b2_hff_init_y(float init_y, float init_ydot) {
  b2_hff_state.y     = init_y;
  b2_hff_state.ydot  = init_ydot;
  int i, j;
  for (i=0; i<HFF_STATE_SIZE; i++) {
    for (j=0; j<HFF_STATE_SIZE; j++)
      b2_hff_state.yP[i][j] = 0.;
	b2_hff_state.yP[i][i] = INIT_PXX;
  }
}

#ifdef GPS_LAG
static inline void b2_hff_store_accel_ltp(float x, float y) {
  past_accel[acc_buf_w].x = x;
  past_accel[acc_buf_w].y = y;
  INC_ACC_IDX(acc_buf_w);

  if (acc_buf_n < ACC_BUF_MAXN) {
	acc_buf_n++;
  } else {
	INC_ACC_IDX(acc_buf_r);
  }
}

/* get the accel values from back_n steps ago */
static inline void b2_hff_get_past_accel(unsigned int back_n) {
  int i;
  if (back_n > acc_buf_n) {
	PRINT_DBG(1, ("Cannot go back %d steps, going back only %d instead!\n", back_n, acc_buf_n));
	back_n = acc_buf_n;
  } else if (back_n == 0) {
	PRINT_DBG(1, ("Cannot go back zero steps!\n"));
	return;
  }
  if ((int)(acc_buf_w - back_n) < 0)
	i = acc_buf_w - back_n  + ACC_BUF_MAXN;
  else
	i = acc_buf_w - back_n;
  b2_hff_xdd_meas = past_accel[i].x;
  b2_hff_ydd_meas = past_accel[i].y;
  PRINT_DBG(3, ("get past accel. buf_n: %2d \tbuf_w: %2d \tback_n: %2d \ti: %2d \txdd: %f \tydd: %f\n", acc_buf_n, acc_buf_w, back_n, i, b2_hff_xdd_meas, b2_hff_ydd_meas));
}

static inline void b2_hff_rb_put_state(struct HfilterFloat* source) {
  /* copy state from source into buffer */
  b2_hff_set_state(b2_hff_rb_put, source);
  b2_hff_rb_put->lag_counter = 0;
  b2_hff_rb_put->rollback = FALSE;

  /* forward write pointer */
  INC_RB_POINTER(b2_hff_rb_put);

  /* increase fill count and forward last pointer if neccessary */
  if (b2_hff_rb_n < HFF_RB_MAXN) {
	b2_hff_rb_n++;
  } else {
	INC_RB_POINTER(b2_hff_rb_last);
  }
  PRINT_DBG(2, ("put state. fill count now: %d\n", b2_hff_rb_n));
}

static inline void b2_hff_rb_drop_last(void) {
  if (b2_hff_rb_n > 0) {
	INC_RB_POINTER(b2_hff_rb_last);
	b2_hff_rb_n--;
  } else {
	PRINT_DBG(2, ("hff ringbuffer empty!\n"));
	b2_hff_rb_last->lag_counter = 0;
	b2_hff_rb_last->rollback = FALSE;
  }
  PRINT_DBG(2, ("drop last state. fill count now: %d\n", b2_hff_rb_n));
}


/* copy source state to dest state */
static inline void b2_hff_set_state(struct HfilterFloat* dest, struct HfilterFloat* source) {
  dest->x       = source->x;
  dest->xdot    = source->xdot;
  dest->xdotdot = source->xdotdot;
  dest->y       = source->y;
  dest->ydot    = source->ydot;
  dest->ydotdot = source->ydotdot;
  for (int i=0; i < HFF_STATE_SIZE; i++) {
	for (int j=0; j < HFF_STATE_SIZE; j++) {
	  dest->xP[i][j] = source->xP[i][j];
	  dest->yP[i][j] = source->yP[i][j];
	}
  }
}

static inline void b2_hff_propagate_past(struct HfilterFloat* hff_past) {
  PRINT_DBG(1, ("enter propagate past: %d\n", hff_past->lag_counter));
  /* run max MAX_PP_STEPS propagation steps */
  for (int i=0; i < MAX_PP_STEPS; i++) {
	if (hff_past->lag_counter > 0) {
	  b2_hff_get_past_accel(hff_past->lag_counter);
	  PRINT_DBG(2, ("propagate past: %d\n", hff_past->lag_counter));
	  b2_hff_propagate_x(hff_past);
	  b2_hff_propagate_y(hff_past);
	  hff_past->lag_counter--;

	  if (past_save_counter > 0) {
		past_save_counter--;
		PRINT_DBG(2, ("dec past_save_counter: %d\n", past_save_counter));
	  } else if (past_save_counter == SAVE_NOW) {
		/* next GPS measurement valid at this state -> save */
		PRINT_DBG(2, ("save past state\n"));
		b2_hff_rb_put_state(hff_past);
		past_save_counter = SAVING;
	  } else if (past_save_counter == SAVING) {
		/* increase lag counter on if next state is already saved */
		if (hff_past == &b2_hff_rb[HFF_RB_MAXN-1])
		  b2_hff_rb[0].lag_counter++;
		else
		  (hff_past+1)->lag_counter++;
	  }
	}

	/* finished re-propagating the past values */
	if (hff_past->lag_counter == 0) {
	  b2_hff_set_state(&b2_hff_state, hff_past);
	  b2_hff_rb_drop_last();
      past_save_counter = SAVE_DONE;
	  break;
	}
  }
}
#endif /* GPS_LAG */



void b2_hff_propagate(void) {
  if (b2_hff_lost_counter < b2_hff_lost_limit)
    b2_hff_lost_counter++;

#ifdef GPS_LAG
  /* continue re-propagating to catch up with the present */
  if (b2_hff_rb_last->rollback) {
	b2_hff_propagate_past(b2_hff_rb_last);
  }
#endif

  /* store body accelerations for mean computation */
  b2_hff_store_accel_body();

  /* propagate current state if it is time */
  if (b2_hff_ps_counter == HFF_PRESCALER) {
	b2_hff_ps_counter = 1;

    if (b2_hff_lost_counter < b2_hff_lost_limit) {
      /* compute float ltp mean acceleration */
      b2_hff_compute_accel_body_mean(HFF_PRESCALER);
      struct Int32Vect3 mean_accel_ltp;
      INT32_RMAT_TRANSP_VMULT(mean_accel_ltp, ahrs.ltp_to_body_rmat, acc_body_mean);
      b2_hff_xdd_meas = ACCEL_FLOAT_OF_BFP(mean_accel_ltp.x);
      b2_hff_ydd_meas = ACCEL_FLOAT_OF_BFP(mean_accel_ltp.y);
#ifdef GPS_LAG
      b2_hff_store_accel_ltp(b2_hff_xdd_meas, b2_hff_ydd_meas);
#endif

      /*
       * propagate current state
       */
      b2_hff_propagate_x(&b2_hff_state);
      b2_hff_propagate_y(&b2_hff_state);

      /* update ins state from horizontal filter */
      ins_ltp_accel.x = ACCEL_BFP_OF_REAL(b2_hff_state.xdotdot);
      ins_ltp_accel.y = ACCEL_BFP_OF_REAL(b2_hff_state.ydotdot);
      ins_ltp_speed.x = SPEED_BFP_OF_REAL(b2_hff_state.xdot);
      ins_ltp_speed.y = SPEED_BFP_OF_REAL(b2_hff_state.ydot);
      ins_ltp_pos.x   = POS_BFP_OF_REAL(b2_hff_state.x);
      ins_ltp_pos.y   = POS_BFP_OF_REAL(b2_hff_state.y);

#ifdef GPS_LAG
      /* increase lag counter on last saved state */
      if (b2_hff_rb_n > 0)
        b2_hff_rb_last->lag_counter++;

      /* save filter state if needed */
      if (save_counter == 0) {
        PRINT_DBG(1, ("save current state\n"));
        b2_hff_rb_put_state(&b2_hff_state);
        save_counter = -1;
      } else if (save_counter > 0) {
        save_counter--;
      }
#endif
    }
  } else {
    b2_hff_ps_counter++;
  }
}




void b2_hff_update_gps(void) {
  b2_hff_lost_counter = 0;

#if USE_GPS_ACC4R
  Rgps_pos = (float) gps.pacc / 100.;
  if (Rgps_pos < HFF_R_POS_MIN)
    Rgps_pos = HFF_R_POS_MIN;

  Rgps_vel = (float) gps.sacc / 100.;
  if (Rgps_vel < HFF_R_SPEED_MIN)
    Rgps_vel = HFF_R_SPEED_MIN;
#endif

#ifdef GPS_LAG
  if (GPS_LAG_N == 0) {
#endif

    /* update filter state with measurement */
    b2_hff_update_x(&b2_hff_state, ins_gps_pos_m_ned.x, Rgps_pos);
    b2_hff_update_y(&b2_hff_state, ins_gps_pos_m_ned.y, Rgps_pos);
#ifdef HFF_UPDATE_SPEED
    b2_hff_update_xdot(&b2_hff_state, ins_gps_speed_m_s_ned.x, Rgps_vel);
    b2_hff_update_ydot(&b2_hff_state, ins_gps_speed_m_s_ned.y, Rgps_vel);
#endif

    /* update ins state */
    ins_ltp_accel.x = ACCEL_BFP_OF_REAL(b2_hff_state.xdotdot);
    ins_ltp_accel.y = ACCEL_BFP_OF_REAL(b2_hff_state.ydotdot);
    ins_ltp_speed.x = SPEED_BFP_OF_REAL(b2_hff_state.xdot);
    ins_ltp_speed.y = SPEED_BFP_OF_REAL(b2_hff_state.ydot);
    ins_ltp_pos.x   = POS_BFP_OF_REAL(b2_hff_state.x);
    ins_ltp_pos.y   = POS_BFP_OF_REAL(b2_hff_state.y);

#ifdef GPS_LAG
  } else if (b2_hff_rb_n > 0) {
    /* roll back if state was saved approx when GPS was valid */
    lag_counter_err = b2_hff_rb_last->lag_counter - GPS_LAG_N;
    PRINT_DBG(2, ("update. rb_n: %d  lag_counter: %d  lag_cnt_err: %d\n", b2_hff_rb_n, b2_hff_rb_last->lag_counter, lag_counter_err));
    if (abs(lag_counter_err) <= GPS_LAG_TOL_N) {
      b2_hff_rb_last->rollback = TRUE;
      b2_hff_update_x(b2_hff_rb_last, ins_gps_pos_m_ned.x, Rgps_pos);
      b2_hff_update_y(b2_hff_rb_last, ins_gps_pos_m_ned.y, Rgps_pos);
#ifdef HFF_UPDATE_SPEED
      b2_hff_update_xdot(b2_hff_rb_last, ins_gps_speed_m_s_ned.x, Rgps_vel);
      b2_hff_update_ydot(b2_hff_rb_last, ins_gps_speed_m_s_ned.y, Rgps_vel);
#endif
      past_save_counter = GPS_DT_N-1;// + lag_counter_err;
      PRINT_DBG(2, ("gps updated. past_save_counter: %d\n", past_save_counter));
      b2_hff_propagate_past(b2_hff_rb_last);
    } else if (lag_counter_err >= GPS_DT_N - (GPS_LAG_TOL_N+1)) {
      /* apparently missed a GPS update, try next saved state */
      PRINT_DBG(2, ("try next saved state\n"));
      b2_hff_rb_drop_last();
      b2_hff_update_gps();
    }
  } else if (save_counter < 0) {
    /* ringbuffer empty -> save output filter state at next GPS validity point in time */
    save_counter = GPS_DT_N-1 - (GPS_LAG_N % GPS_DT_N);
    PRINT_DBG(2, ("rb empty, save counter set: %d\n", save_counter));
  }

#endif /* GPS_LAG */
}


void b2_hff_realign(struct FloatVect2 pos, struct FloatVect2 vel) {
  b2_hff_state.x = pos.x;
  b2_hff_state.y = pos.y;
  b2_hff_state.xdot = vel.x;
  b2_hff_state.ydot = vel.y;
#ifdef GPS_LAG
  while (b2_hff_rb_n > 0) {
	b2_hff_rb_drop_last();
  }
  save_counter = -1;
  past_save_counter = SAVE_DONE;
#endif
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
  hff_work->xdotdot = b2_hff_xdd_meas;
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
  hff_work->ydotdot = b2_hff_ydd_meas;
  hff_work->y = hff_work->y + DT_HFILTER * hff_work->ydot + DT_HFILTER*DT_HFILTER/2 * hff_work->ydotdot;
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
void b2_hff_update_pos (struct FloatVect2 pos, struct FloatVect2 Rpos) {
  b2_hff_update_x(&b2_hff_state, pos.x, Rpos.x);
  b2_hff_update_y(&b2_hff_state, pos.y, Rpos.y);
}

static inline void b2_hff_update_x(struct HfilterFloat* hff_work, float x_meas, float Rpos) {
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

static inline void b2_hff_update_y(struct HfilterFloat* hff_work, float y_meas, float Rpos) {
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
 * Update velocity
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
void b2_hff_update_vel(struct FloatVect2 vel, struct FloatVect2 Rvel) {
  b2_hff_update_xdot(&b2_hff_state, vel.x, Rvel.x);
  b2_hff_update_ydot(&b2_hff_state, vel.y, Rvel.y);
}

static inline void b2_hff_update_xdot(struct HfilterFloat* hff_work, float vel, float Rvel) {
  b2_hff_xd_meas = vel;

  const float yd = vel - hff_work->xdot;
  const float S  = hff_work->xP[1][1] + Rvel;
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

static inline void b2_hff_update_ydot(struct HfilterFloat* hff_work, float vel, float Rvel) {
  b2_hff_yd_meas = vel;

  const float yd = vel - hff_work->ydot;
  const float S  = hff_work->yP[1][1] + Rvel;
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
