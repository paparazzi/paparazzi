/*
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

/**
 * @file subsystems/ins/hf_float.c
 *
 * Horizontal filter (x,y) to estimate position and velocity.
 *
 */

#include "subsystems/ins/hf_float.h"
#include "subsystems/imu.h"
#include "state.h"
#include "subsystems/gps.h"
#include <stdlib.h>
#include "filters/low_pass_filter.h"
#include "generated/airframe.h"

#ifdef SITL
#include <stdio.h>
#define DBG_LEVEL 1
#define PRINT_DBG(_l, _p) {           \
    if (DBG_LEVEL >= _l)            \
      printf _p;                \
  }
#else
#define PRINT_DBG(_l, _p) {}
#endif


#ifndef AHRS_PROPAGATE_FREQUENCY
#define AHRS_PROPAGATE_FREQUENCY PERIODIC_FREQUENCY
#endif

#ifndef HFF_PRESCALER
#if AHRS_PROPAGATE_FREQUENCY == 512
#define HFF_PRESCALER 16
#elif AHRS_PROPAGATE_FREQUENCY == 500
#define HFF_PRESCALER 10
#else
#error "HFF_PRESCALER not set, needs to be a divisor of AHRS_PROPAGATE_FREQUENCY"
#endif
#endif

/** horizontal filter propagation frequency */
#define HFF_FREQ ((AHRS_PROPAGATE_FREQUENCY)/HFF_PRESCALER)
#define DT_HFILTER (1./HFF_FREQ)

/** initial covariance diagonal */
#define INIT_PXX 1.
/** process noise (is the same for x and y)*/
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

#ifndef HFF_UPDATE_SPEED
#define HFF_UPDATE_SPEED TRUE
#endif

#ifndef HFF_LOWPASS_CUTOFF_FREQUENCY
#define HFF_LOWPASS_CUTOFF_FREQUENCY 14
#endif

#if HFF_LOWPASS_CUTOFF_FREQUENCY < 8
#error "It is not allowed to use a cutoff frequency lower than 8Hz due to overflow issues."
#endif

/* low pass filter variables */
Butterworth2LowPass_int filter_x;
Butterworth2LowPass_int filter_y;
Butterworth2LowPass_int filter_z;

/* gps measurement noise */
float Rgps_pos, Rgps_vel;

/*

  X_x = [ x xdot]
  X_y = [ y ydot]


*/
/* output filter states */
struct HfilterFloat b2_hff_state;


/* last acceleration measurement */
static float b2_hff_xdd_meas;
static float b2_hff_ydd_meas;

/* last velocity measurement */
static float b2_hff_xd_meas;
static float b2_hff_yd_meas;

/* last position measurement */
static float b2_hff_x_meas;
static float b2_hff_y_meas;

/** counter for hff propagation*/
static int b2_hff_ps_counter;

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

/** number of propagaton steps to redo according to GPS_LAG */
#define GPS_LAG_N ((int) (GPS_LAG * HFF_FREQ + 0.5))
/** number of propagation steps between two GPS updates */
#define GPS_DT_N ((int) (HFF_FREQ / 4))
/** tolerance of the GPS lag accuracy is +- GPS_LAG_TOLERANCE seconds */
#define GPS_LAG_TOLERANCE 0.08
#define GPS_LAG_TOL_N ((int) (GPS_LAG_TOLERANCE * HFF_FREQ + 0.5))

/* maximum number of past propagation steps to rerun per ap cycle
 * make sure GPS_LAG_N/MAX_PP_STEPS < 128
 * GPS_LAG_N/MAX_PP_STEPS/512 = seconds until re-propagation catches up with the present
 */
#define MAX_PP_STEPS 6

/* variables for mean accel buffer */
#define ACC_BUF_MAXN (GPS_LAG_N+10)
#define INC_ACC_IDX(idx) {  idx = (idx + 1) < ACC_BUF_MAXN ? (idx + 1) : 0; }

/** buffer with past mean accel values for redoing the propagation */
struct FloatVect2 past_accel[ACC_BUF_MAXN];

static unsigned int acc_buf_r; ///< pos to read from, oldest measurement
static unsigned int acc_buf_w; ///< pos to write to
static unsigned int acc_buf_n; ///< number of elements in buffer


/*
 * stuff for ringbuffer to store past filter states
 */
#define HFF_RB_MAXN ((int) (GPS_LAG * 4))
#define INC_RB_POINTER(ptr) {         \
    if (ptr == &b2_hff_rb[HFF_RB_MAXN-1])   \
      ptr = b2_hff_rb;              \
    else                    \
      ptr++;                  \
  }

/** ringbuffer with state and covariance when GPS was valid */
struct HfilterFloat b2_hff_rb[HFF_RB_MAXN];
struct HfilterFloat *b2_hff_rb_put; ///< ringbuffer write pointer
#endif /* GPS_LAG */

struct HfilterFloat *b2_hff_rb_last; ///< ringbuffer read pointer
static int b2_hff_rb_n; ///< ringbuffer fill count


/** by how many steps the estimated GPS validity point in time differed from GPS_LAG_N */
static int lag_counter_err;

/** counts down the propagation steps until the filter state is saved again */
static int save_counter;
static int past_save_counter;
#define SAVE_NOW 0
#define SAVING -1
#define SAVE_DONE -2

#define HFF_LOST_LIMIT 1000
static uint16_t b2_hff_lost_limit;
static uint16_t b2_hff_lost_counter;

#ifdef GPS_LAG
static void b2_hff_get_past_accel(unsigned int back_n);
static void b2_hff_rb_put_state(struct HfilterFloat *source);
static void b2_hff_rb_drop_last(void);
static void b2_hff_set_state(struct HfilterFloat *dest, struct HfilterFloat *source);
#endif


static void b2_hff_init_x(float init_x, float init_xdot);
static void b2_hff_init_y(float init_y, float init_ydot);

static void b2_hff_propagate_x(struct HfilterFloat *hff_work, float dt);
static void b2_hff_propagate_y(struct HfilterFloat *hff_work, float dt);

static void b2_hff_update_x(struct HfilterFloat *hff_work, float x_meas, float Rpos);
static void b2_hff_update_y(struct HfilterFloat *hff_work, float y_meas, float Rpos);

static void b2_hff_update_xdot(struct HfilterFloat *hff_work, float vel, float Rvel);
static void b2_hff_update_ydot(struct HfilterFloat *hff_work, float vel, float Rvel);

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_hff(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_HFF(trans, dev, AC_ID,
                    &b2_hff_state.x,
                    &b2_hff_state.y,
                    &b2_hff_state.xdot,
                    &b2_hff_state.ydot,
                    &b2_hff_state.xdotdot,
                    &b2_hff_state.ydotdot);
}

static void send_hff_debug(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_HFF_DBG(trans, dev, AC_ID,
                        &b2_hff_x_meas,
                        &b2_hff_y_meas,
                        &b2_hff_xd_meas,
                        &b2_hff_yd_meas,
                        &b2_hff_state.xP[0][0],
                        &b2_hff_state.yP[0][0],
                        &b2_hff_state.xP[1][1],
                        &b2_hff_state.yP[1][1]);
}

#ifdef GPS_LAG
static void send_hff_gps(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_HFF_GPS(trans, dev, AC_ID,
                        &(b2_hff_rb_last->lag_counter),
                        &lag_counter_err,
                        &save_counter);
}
#endif

#endif

void b2_hff_init(float init_x, float init_xdot, float init_y, float init_ydot)
{
  Rgps_pos = HFF_R_POS;
  Rgps_vel = HFF_R_SPEED;
  b2_hff_init_x(init_x, init_xdot);
  b2_hff_init_y(init_y, init_ydot);
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

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "HFF", send_hff);
  register_periodic_telemetry(DefaultPeriodic, "HFF_DBG", send_hff_debug);
#ifdef GPS_LAG
  register_periodic_telemetry(DefaultPeriodic, "HFF_GPS", send_hff_gps);
#endif
#endif

  init_butterworth_2_low_pass_int(&filter_x, HFF_LOWPASS_CUTOFF_FREQUENCY, (1. / AHRS_PROPAGATE_FREQUENCY), 0);
  init_butterworth_2_low_pass_int(&filter_y, HFF_LOWPASS_CUTOFF_FREQUENCY, (1. / AHRS_PROPAGATE_FREQUENCY), 0);
  init_butterworth_2_low_pass_int(&filter_z, HFF_LOWPASS_CUTOFF_FREQUENCY, (1. / AHRS_PROPAGATE_FREQUENCY), 0);
}

static void b2_hff_init_x(float init_x, float init_xdot)
{
  b2_hff_state.x     = init_x;
  b2_hff_state.xdot  = init_xdot;
  int i, j;
  for (i = 0; i < HFF_STATE_SIZE; i++) {
    for (j = 0; j < HFF_STATE_SIZE; j++) {
      b2_hff_state.xP[i][j] = 0.;
    }
    b2_hff_state.xP[i][i] = INIT_PXX;
  }
}

static void b2_hff_init_y(float init_y, float init_ydot)
{
  b2_hff_state.y     = init_y;
  b2_hff_state.ydot  = init_ydot;
  int i, j;
  for (i = 0; i < HFF_STATE_SIZE; i++) {
    for (j = 0; j < HFF_STATE_SIZE; j++) {
      b2_hff_state.yP[i][j] = 0.;
    }
    b2_hff_state.yP[i][i] = INIT_PXX;
  }
}

#ifdef GPS_LAG
static void b2_hff_store_accel_ltp(float x, float y)
{
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
static void b2_hff_get_past_accel(unsigned int back_n)
{
  int i;
  if (back_n > acc_buf_n) {
    PRINT_DBG(1, ("Cannot go back %d steps, going back only %d instead!\n", back_n, acc_buf_n));
    back_n = acc_buf_n;
  } else if (back_n == 0) {
    PRINT_DBG(1, ("Cannot go back zero steps!\n"));
    return;
  }
  if ((int)(acc_buf_w - back_n) < 0) {
    i = acc_buf_w - back_n  + ACC_BUF_MAXN;
  } else {
    i = acc_buf_w - back_n;
  }
  b2_hff_xdd_meas = past_accel[i].x;
  b2_hff_ydd_meas = past_accel[i].y;
  PRINT_DBG(3, ("get past accel. buf_n: %2d \tbuf_w: %2d \tback_n: %2d \ti: %2d \txdd: %f \tydd: %f\n", acc_buf_n,
                acc_buf_w, back_n, i, b2_hff_xdd_meas, b2_hff_ydd_meas));
}

static void b2_hff_rb_put_state(struct HfilterFloat *source)
{
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

static void b2_hff_rb_drop_last(void)
{
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
static void b2_hff_set_state(struct HfilterFloat *dest, struct HfilterFloat *source)
{
  dest->x       = source->x;
  dest->xdot    = source->xdot;
  dest->xdotdot = source->xdotdot;
  dest->y       = source->y;
  dest->ydot    = source->ydot;
  dest->ydotdot = source->ydotdot;
  for (int i = 0; i < HFF_STATE_SIZE; i++) {
    for (int j = 0; j < HFF_STATE_SIZE; j++) {
      dest->xP[i][j] = source->xP[i][j];
      dest->yP[i][j] = source->yP[i][j];
    }
  }
}

static void b2_hff_propagate_past(struct HfilterFloat *hff_past)
{
  PRINT_DBG(1, ("enter propagate past: %d\n", hff_past->lag_counter));
  /* run max MAX_PP_STEPS propagation steps */
  for (int i = 0; i < MAX_PP_STEPS; i++) {
    if (hff_past->lag_counter > 0) {
      b2_hff_get_past_accel(hff_past->lag_counter);
      PRINT_DBG(2, ("propagate past: %d\n", hff_past->lag_counter));
      b2_hff_propagate_x(hff_past, DT_HFILTER);
      b2_hff_propagate_y(hff_past, DT_HFILTER);
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
        if (hff_past == &b2_hff_rb[HFF_RB_MAXN - 1]) {
          b2_hff_rb[0].lag_counter++;
        } else {
          (hff_past + 1)->lag_counter++;
        }
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


void b2_hff_propagate(void)
{
  if (b2_hff_lost_counter < b2_hff_lost_limit) {
    b2_hff_lost_counter++;
  }

#ifdef GPS_LAG
  /* continue re-propagating to catch up with the present */
  if (b2_hff_rb_last->rollback) {
    b2_hff_propagate_past(b2_hff_rb_last);
  }
#endif

  /* rotate imu accel measurement to body frame and filter */
  struct Int32Vect3 acc_meas_body;
  struct Int32RMat *body_to_imu_rmat = orientationGetRMat_i(&imu.body_to_imu);
  int32_rmat_transp_vmult(&acc_meas_body, body_to_imu_rmat, &imu.accel);

  struct Int32Vect3 acc_body_filtered;
  acc_body_filtered.x = update_butterworth_2_low_pass_int(&filter_x, acc_meas_body.x);
  acc_body_filtered.y = update_butterworth_2_low_pass_int(&filter_y, acc_meas_body.y);
  acc_body_filtered.z = update_butterworth_2_low_pass_int(&filter_z, acc_meas_body.z);

  /* propagate current state if it is time */
  if (b2_hff_ps_counter == HFF_PRESCALER) {
    b2_hff_ps_counter = 1;
    if (b2_hff_lost_counter < b2_hff_lost_limit) {
      struct Int32Vect3 filtered_accel_ltp;
      struct Int32RMat *ltp_to_body_rmat = stateGetNedToBodyRMat_i();
      int32_rmat_transp_vmult(&filtered_accel_ltp, ltp_to_body_rmat, &acc_body_filtered);
      b2_hff_xdd_meas = ACCEL_FLOAT_OF_BFP(filtered_accel_ltp.x);
      b2_hff_ydd_meas = ACCEL_FLOAT_OF_BFP(filtered_accel_ltp.y);
#ifdef GPS_LAG
      b2_hff_store_accel_ltp(b2_hff_xdd_meas, b2_hff_ydd_meas);
#endif
      /*
       * propagate current state
       */
      b2_hff_propagate_x(&b2_hff_state, DT_HFILTER);
      b2_hff_propagate_y(&b2_hff_state, DT_HFILTER);

#ifdef GPS_LAG
      /* increase lag counter on last saved state */
      if (b2_hff_rb_n > 0) {
        b2_hff_rb_last->lag_counter++;
      }

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

void b2_hff_update_gps(struct FloatVect2 *pos_ned, struct FloatVect2 *speed_ned)
{
  b2_hff_lost_counter = 0;

#if USE_GPS_ACC4R
  Rgps_pos = (float) gps.pacc / 100.;
  if (Rgps_pos < HFF_R_POS_MIN) {
    Rgps_pos = HFF_R_POS_MIN;
  }

  Rgps_vel = (float) gps.sacc / 100.;
  if (Rgps_vel < HFF_R_SPEED_MIN) {
    Rgps_vel = HFF_R_SPEED_MIN;
  }
#endif

#ifdef GPS_LAG
  if (GPS_LAG_N == 0) {
#endif

    /* update filter state with measurement */
    b2_hff_update_x(&b2_hff_state, pos_ned->x, Rgps_pos);
    b2_hff_update_y(&b2_hff_state, pos_ned->y, Rgps_pos);
#if HFF_UPDATE_SPEED
    b2_hff_update_xdot(&b2_hff_state, speed_ned->x, Rgps_vel);
    b2_hff_update_ydot(&b2_hff_state, speed_ned->y, Rgps_vel);
#endif


#ifdef GPS_LAG
  } else if (b2_hff_rb_n > 0) {
    /* roll back if state was saved approx when GPS was valid */
    lag_counter_err = b2_hff_rb_last->lag_counter - GPS_LAG_N;
    PRINT_DBG(2, ("update. rb_n: %d  lag_counter: %d  lag_cnt_err: %d\n", b2_hff_rb_n, b2_hff_rb_last->lag_counter,
                  lag_counter_err));
    if (abs(lag_counter_err) <= GPS_LAG_TOL_N) {
      b2_hff_rb_last->rollback = TRUE;
      b2_hff_update_x(b2_hff_rb_last, pos_ned->x, Rgps_pos);
      b2_hff_update_y(b2_hff_rb_last, pos_ned->y, Rgps_pos);
#if HFF_UPDATE_SPEED
      b2_hff_update_xdot(b2_hff_rb_last, speed_ned->x, Rgps_vel);
      b2_hff_update_ydot(b2_hff_rb_last, speed_ned->y, Rgps_vel);
#endif
      past_save_counter = GPS_DT_N - 1; // + lag_counter_err;
      PRINT_DBG(2, ("gps updated. past_save_counter: %d\n", past_save_counter));
      b2_hff_propagate_past(b2_hff_rb_last);
    } else if (lag_counter_err >= GPS_DT_N - (GPS_LAG_TOL_N + 1)) {
      /* apparently missed a GPS update, try next saved state */
      PRINT_DBG(2, ("try next saved state\n"));
      b2_hff_rb_drop_last();
      b2_hff_update_gps(pos_ned, speed_ned);
    }
  } else if (save_counter < 0) {
    /* ringbuffer empty -> save output filter state at next GPS validity point in time */
    save_counter = GPS_DT_N - 1 - (GPS_LAG_N % GPS_DT_N);
    PRINT_DBG(2, ("rb empty, save counter set: %d\n", save_counter));
  }

#endif /* GPS_LAG */
}


void b2_hff_realign(struct FloatVect2 pos, struct FloatVect2 vel)
{
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
static void b2_hff_propagate_x(struct HfilterFloat *hff_work, float dt)
{
  /* update state */
  hff_work->xdotdot = b2_hff_xdd_meas;
  hff_work->x = hff_work->x + dt * hff_work->xdot + dt * dt / 2 * hff_work->xdotdot;
  hff_work->xdot = hff_work->xdot + dt * hff_work->xdotdot;
  /* update covariance */
  const float FPF00 = hff_work->xP[0][0] + dt * (hff_work->xP[1][0] + hff_work->xP[0][1] + dt * hff_work->xP[1][1]);
  const float FPF01 = hff_work->xP[0][1] + dt * hff_work->xP[1][1];
  const float FPF10 = hff_work->xP[1][0] + dt * hff_work->xP[1][1];
  const float FPF11 = hff_work->xP[1][1];

  hff_work->xP[0][0] = FPF00 + Q;
  hff_work->xP[0][1] = FPF01;
  hff_work->xP[1][0] = FPF10;
  hff_work->xP[1][1] = FPF11 + Qdotdot;
}

static void b2_hff_propagate_y(struct HfilterFloat *hff_work, float dt)
{
  /* update state */
  hff_work->ydotdot = b2_hff_ydd_meas;
  hff_work->y = hff_work->y + dt * hff_work->ydot + dt * dt / 2 * hff_work->ydotdot;
  hff_work->ydot = hff_work->ydot + dt * hff_work->ydotdot;
  /* update covariance */
  const float FPF00 = hff_work->yP[0][0] + dt * (hff_work->yP[1][0] + hff_work->yP[0][1] + dt * hff_work->yP[1][1]);
  const float FPF01 = hff_work->yP[0][1] + dt * hff_work->yP[1][1];
  const float FPF10 = hff_work->yP[1][0] + dt * hff_work->yP[1][1];
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
void b2_hff_update_pos(struct FloatVect2 pos, struct FloatVect2 Rpos)
{
  b2_hff_update_x(&b2_hff_state, pos.x, Rpos.x);
  b2_hff_update_y(&b2_hff_state, pos.y, Rpos.y);
}

static void b2_hff_update_x(struct HfilterFloat *hff_work, float x_meas, float Rpos)
{
  b2_hff_x_meas = x_meas;

  const float y  = x_meas - hff_work->x;
  const float S  = hff_work->xP[0][0] + Rpos;
  const float K1 = hff_work->xP[0][0] * 1 / S;
  const float K2 = hff_work->xP[1][0] * 1 / S;

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

static void b2_hff_update_y(struct HfilterFloat *hff_work, float y_meas, float Rpos)
{
  b2_hff_y_meas = y_meas;

  const float y  = y_meas - hff_work->y;
  const float S  = hff_work->yP[0][0] + Rpos;
  const float K1 = hff_work->yP[0][0] * 1 / S;
  const float K2 = hff_work->yP[1][0] * 1 / S;

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
void b2_hff_update_vel(struct FloatVect2 vel, struct FloatVect2 Rvel)
{
  b2_hff_update_xdot(&b2_hff_state, vel.x, Rvel.x);
  b2_hff_update_ydot(&b2_hff_state, vel.y, Rvel.y);
}

static void b2_hff_update_xdot(struct HfilterFloat *hff_work, float vel, float Rvel)
{
  b2_hff_xd_meas = vel;

  const float yd = vel - hff_work->xdot;
  const float S  = hff_work->xP[1][1] + Rvel;
  const float K1 = hff_work->xP[0][1] * 1 / S;
  const float K2 = hff_work->xP[1][1] * 1 / S;

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

static void b2_hff_update_ydot(struct HfilterFloat *hff_work, float vel, float Rvel)
{
  b2_hff_yd_meas = vel;

  const float yd = vel - hff_work->ydot;
  const float S  = hff_work->yP[1][1] + Rvel;
  const float K1 = hff_work->yP[0][1] * 1 / S;
  const float K2 = hff_work->yP[1][1] * 1 / S;

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
