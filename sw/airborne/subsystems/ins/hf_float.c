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
#include "modules/imu/imu.h"
#include "state.h"
#include "modules/gps/gps.h"
#include <stdlib.h>
#include "filters/low_pass_filter.h"
#include "generated/airframe.h"

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
#endif

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
#elif AHRS_PROPAGATE_FREQUENCY == 200
#define HFF_PRESCALER 6
#else
#error "HFF_PRESCALER not set, needs to be a divisor of AHRS_PROPAGATE_FREQUENCY"
#endif
#endif

/** horizontal filter propagation frequency */
#define HFF_FREQ (AHRS_PROPAGATE_FREQUENCY / HFF_PRESCALER)
#define HFF_DT (1./HFF_FREQ)

/** initial covariance diagonal */
#define HFF_INIT_PXX 1.
/** process noise (is the same for x and y)*/
#ifndef HFF_ACCEL_NOISE
#define HFF_ACCEL_NOISE 0.5
#endif
#define HFF_Q       HFF_ACCEL_NOISE
#define HFF_Qdotdot HFF_ACCEL_NOISE

//TODO: proper measurement noise
#ifndef HFF_R_POS
#define HFF_R_POS   8.
#endif
#ifndef HFF_R_POS_MIN
#define HFF_R_POS_MIN 3.
#endif

#ifndef HFF_R_GPS_SPEED
#define HFF_R_GPS_SPEED 2.
#endif
#ifndef HFF_R_GPS_SPEED_MIN
#define HFF_R_GPS_SPEED_MIN 0.25
#endif

#ifndef HFF_UPDATE_GPS_SPEED
#define HFF_UPDATE_GPS_SPEED TRUE
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
  X_x = [ x xdot xbias ]
  X_y = [ y ydot ybias ]
*/

/* output filter states */
struct HfilterFloat hff;

/* last acceleration measurement */
static float hff_xdd_meas = 0;
static float hff_ydd_meas = 0;

/* last velocity measurement */
static float hff_xd_meas = 0;
static float hff_yd_meas = 0;

/* last position measurement */
static float hff_x_meas = 0;
static float hff_y_meas = 0;

/** counter for hff propagation*/
static int hff_ps_counter;

/* default parameters */
#define HFF_Qbiasbias 1e-7

/*
 * For GPS lag compensation
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
    if (ptr == &hff_rb[HFF_RB_MAXN-1])   \
      ptr = hff_rb;              \
    else                    \
      ptr++;                  \
  }

/** ringbuffer with state and covariance when GPS was valid */
struct HfilterFloat hff_rb[HFF_RB_MAXN];
struct HfilterFloat *hff_rb_put; ///< ringbuffer write pointer
#endif /* GPS_LAG */

struct HfilterFloat *hff_rb_last; ///< ringbuffer read pointer
static int hff_rb_n; ///< ringbuffer fill count


/** by how many steps the estimated GPS validity point in time differed from GPS_LAG_N */
static int16_t lag_counter_err;

/** counts down the propagation steps until the filter state is saved again */
static int16_t save_counter;
static int past_save_counter;
#define SAVE_NOW 0
#define SAVING -1
#define SAVE_DONE -2

#define HFF_LOST_LIMIT 1000
static uint16_t hff_lost_limit;
static uint16_t hff_lost_counter, hff_speed_lost_counter;

#ifdef GPS_LAG
static void hff_get_past_accel(unsigned int back_n);
static void hff_rb_put_state(struct HfilterFloat *source);
static void hff_rb_drop_last(void);
static void hff_set_state(struct HfilterFloat *dest, struct HfilterFloat *source);
#endif

static void hff_init_x(float init_x, float init_xdot, float init_xbias);
static void hff_init_y(float init_y, float init_ydot, float init_ybias);

static void hff_propagate_x(struct HfilterFloat *filt, float dt);
static void hff_propagate_y(struct HfilterFloat *filt, float dt);

static void hff_update_x(struct HfilterFloat *filt, float x_meas, float Rpos);
static void hff_update_y(struct HfilterFloat *filt, float y_meas, float Rpos);

static void hff_update_xdot(struct HfilterFloat *filt, float vel, float Rvel);
static void hff_update_ydot(struct HfilterFloat *filt, float vel, float Rvel);

#if PERIODIC_TELEMETRY

static void send_hff(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_HFF(trans, dev, AC_ID,
                    &hff.x,
                    &hff.y,
                    &hff.xdot,
                    &hff.ydot,
                    &hff.xdotdot,
                    &hff.ydotdot,
                    &hff.xbias,
                    &hff.ybias);
}

static void send_hff_debug(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_HFF_DBG(trans, dev, AC_ID,
                        &hff_x_meas,
                        &hff_y_meas,
                        &hff_xd_meas,
                        &hff_yd_meas,
                        &hff.xP[0][0],
                        &hff.yP[0][0],
                        &hff.xP[1][1],
                        &hff.yP[1][1],
                        &hff.xP[2][2],
                        &hff.yP[2][2]);
}

#ifdef GPS_LAG
static void send_hff_gps(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_HFF_GPS(trans, dev, AC_ID,
                        &(hff_rb_last->lag_counter),
                        &lag_counter_err,
                        &save_counter);
}
#endif

#endif

void hff_init(float init_x, float init_xdot, float init_y, float init_ydot)
{
  Rgps_pos = HFF_R_POS;
  Rgps_vel = HFF_R_GPS_SPEED;
  hff_init_x(init_x, init_xdot, 0.f);
  hff_init_y(init_y, init_ydot, 0.f);
#ifdef GPS_LAG
  /* init buffer for past mean accel values */
  acc_buf_r = 0;
  acc_buf_w = 0;
  acc_buf_n = 0;
  hff_rb_put = hff_rb;
  hff_rb_last = hff_rb;
  hff_rb_last->rollback = false;
  hff_rb_last->lag_counter = 0;
  hff.lag_counter = GPS_LAG_N;
#ifdef SITL
  printf("GPS_LAG: %f\n", GPS_LAG);
  printf("GPS_LAG_N: %d\n", GPS_LAG_N);
  printf("GPS_DT_N: %d\n", GPS_DT_N);
  printf("HFF_DT: %f\n", HFF_DT);
  printf("GPS_LAG_TOL_N: %i\n", GPS_LAG_TOL_N);
#endif
#else
  hff_rb_last = &hff;
  hff.lag_counter = 0;
#endif
  hff_rb_n = 0;
  hff.rollback = false;
  lag_counter_err = 0;
  save_counter = -1;
  past_save_counter = SAVE_DONE;
  hff_lost_counter = 0;
  hff_speed_lost_counter = 0;
  hff_lost_limit = HFF_LOST_LIMIT;
  hff_ps_counter = 0;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_HFF, send_hff);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_HFF_DBG, send_hff_debug);
#ifdef GPS_LAG
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_HFF_GPS, send_hff_gps);
#endif
#endif

  init_butterworth_2_low_pass_int(&filter_x, HFF_LOWPASS_CUTOFF_FREQUENCY, (1. / AHRS_PROPAGATE_FREQUENCY), 0);
  init_butterworth_2_low_pass_int(&filter_y, HFF_LOWPASS_CUTOFF_FREQUENCY, (1. / AHRS_PROPAGATE_FREQUENCY), 0);
  init_butterworth_2_low_pass_int(&filter_z, HFF_LOWPASS_CUTOFF_FREQUENCY, (1. / AHRS_PROPAGATE_FREQUENCY), 0);
}

static void hff_init_x(float init_x, float init_xdot, float init_xbias)
{
  hff.x     = init_x;
  hff.xdot  = init_xdot;
  hff.xbias = init_xbias;
  int i, j;
  for (i = 0; i < HFF_STATE_SIZE; i++) {
    for (j = 0; j < HFF_STATE_SIZE; j++) {
      hff.xP[i][j] = 0.;
    }
    hff.xP[i][i] = HFF_INIT_PXX;
  }
}

static void hff_init_y(float init_y, float init_ydot, float init_ybias)
{
  hff.y     = init_y;
  hff.ydot  = init_ydot;
  hff.ybias = init_ybias;
  int i, j;
  for (i = 0; i < HFF_STATE_SIZE; i++) {
    for (j = 0; j < HFF_STATE_SIZE; j++) {
      hff.yP[i][j] = 0.;
    }
    hff.yP[i][i] = HFF_INIT_PXX;
  }
}

#ifdef GPS_LAG
static void hff_store_accel_ltp(float x, float y)
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
static void hff_get_past_accel(unsigned int back_n)
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
  hff_xdd_meas = past_accel[i].x;
  hff_ydd_meas = past_accel[i].y;
  PRINT_DBG(3, ("get past accel. buf_n: %2d \tbuf_w: %2d \tback_n: %2d \ti: %2d \txdd: %f \tydd: %f\n", acc_buf_n,
                acc_buf_w, back_n, i, hff_xdd_meas, hff_ydd_meas));
}

static void hff_rb_put_state(struct HfilterFloat *source)
{
  /* copy state from source into buffer */
  hff_set_state(hff_rb_put, source);
  hff_rb_put->lag_counter = 0;
  hff_rb_put->rollback = false;

  /* forward write pointer */
  INC_RB_POINTER(hff_rb_put);

  /* increase fill count and forward last pointer if neccessary */
  if (hff_rb_n < HFF_RB_MAXN) {
    hff_rb_n++;
  } else {
    INC_RB_POINTER(hff_rb_last);
  }
  PRINT_DBG(2, ("put state. fill count now: %d\n", hff_rb_n));
}

static void hff_rb_drop_last(void)
{
  if (hff_rb_n > 0) {
    INC_RB_POINTER(hff_rb_last);
    hff_rb_n--;
  } else {
    PRINT_DBG(2, ("hff ringbuffer empty!\n"));
    hff_rb_last->lag_counter = 0;
    hff_rb_last->rollback = false;
  }
  PRINT_DBG(2, ("drop last state. fill count now: %d\n", hff_rb_n));
}

/* copy source state to dest state */
static void hff_set_state(struct HfilterFloat *dest, struct HfilterFloat *source)
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

static void hff_propagate_past(struct HfilterFloat *filt_past)
{
  PRINT_DBG(1, ("enter propagate past: %d\n", hff_past->lag_counter));
  /* run max MAX_PP_STEPS propagation steps */
  for (int i = 0; i < MAX_PP_STEPS; i++) {
    if (hff_past->lag_counter > 0) {
      hff_get_past_accel(hff_past->lag_counter);
      PRINT_DBG(2, ("propagate past: %d\n", hff_past->lag_counter));
      hff_propagate_x(hff_past, HFF_DT);
      hff_propagate_y(hff_past, HFF_DT);
      hff_past->lag_counter--;

      if (past_save_counter > 0) {
        past_save_counter--;
        PRINT_DBG(2, ("dec past_save_counter: %d\n", past_save_counter));
      } else if (past_save_counter == SAVE_NOW) {
        /* next GPS measurement valid at this state -> save */
        PRINT_DBG(2, ("save past state\n"));
        hff_rb_put_state(hff_past);
        past_save_counter = SAVING;
      } else if (past_save_counter == SAVING) {
        /* increase lag counter on if next state is already saved */
        if (hff_past == &hff_rb[HFF_RB_MAXN - 1]) {
          hff_rb[0].lag_counter++;
        } else {
          (hff_past + 1)->lag_counter++;
        }
      }
    }

    /* finished re-propagating the past values */
    if (hff_past->lag_counter == 0) {
      hff_set_state(&hff, hff_past);
      hff_rb_drop_last();
      past_save_counter = SAVE_DONE;
      break;
    }
  }
}
#endif /* GPS_LAG */


void hff_propagate(void)
{
  if (hff_lost_counter < hff_lost_limit) {
    hff_lost_counter++;
  }

  if (hff_speed_lost_counter < hff_lost_limit) {
    hff_speed_lost_counter++;
  }

#ifdef GPS_LAG
  /* continue re-propagating to catch up with the present */
  if (hff_rb_last->rollback) {
    hff_propagate_past(hff_rb_last);
  }
#endif

  /* rotate imu accel measurement to body frame and filter */
  struct Int32Vect3 acc_body_filtered;
  acc_body_filtered.x = update_butterworth_2_low_pass_int(&filter_x, stateGetAccelBody_i()->x);
  acc_body_filtered.y = update_butterworth_2_low_pass_int(&filter_y, stateGetAccelBody_i()->y);
  acc_body_filtered.z = update_butterworth_2_low_pass_int(&filter_z, stateGetAccelBody_i()->z);

  /* propagate current state if it is time */
  if (hff_ps_counter >= HFF_PRESCALER) {
    hff_ps_counter = 0;
    struct Int32Vect3 filtered_accel_ltp;
    struct Int32RMat *ltp_to_body_rmat = stateGetNedToBodyRMat_i();
    int32_rmat_transp_vmult(&filtered_accel_ltp, ltp_to_body_rmat, &acc_body_filtered);
    hff_xdd_meas = ACCEL_FLOAT_OF_BFP(filtered_accel_ltp.x);
    hff_ydd_meas = ACCEL_FLOAT_OF_BFP(filtered_accel_ltp.y);

#ifdef GPS_LAG
    hff_store_accel_ltp(hff_xdd_meas, hff_ydd_meas);
#endif
    if (hff_lost_counter < hff_lost_limit || hff_speed_lost_counter < hff_lost_limit) {
      /*
       * propagate current state
       */
      hff_propagate_x(&hff, HFF_DT);
      hff_propagate_y(&hff, HFF_DT);

#ifdef GPS_LAG
      /* increase lag counter on last saved state */
      if (hff_rb_n > 0) {
        hff_rb_last->lag_counter++;
      }

      /* save filter state if needed */
      if (save_counter == 0) {
        PRINT_DBG(1, ("save current state\n"));
        hff_rb_put_state(&hff);
        save_counter = -1;
      } else if (save_counter > 0) {
        save_counter--;
      }
#endif
    }
  }
  hff_ps_counter++;
}

void hff_update_gps(struct FloatVect2 *pos_ned, struct FloatVect2 *speed_ned __attribute__((unused)))
{
  hff_lost_counter = 0;

#if USE_GPS_ACC4R
  Rgps_pos = (float) gps.pacc / 100.;
  if (Rgps_pos < HFF_R_POS_MIN) {
    Rgps_pos = HFF_R_POS_MIN;
  }

  Rgps_vel = (float) gps.sacc / 100.;
  if (Rgps_vel < HFF_R_GPS_SPEED_MIN) {
    Rgps_vel = HFF_R_GPS_SPEED_MIN;
  }
#endif

#ifdef GPS_LAG
  if (GPS_LAG_N == 0) {
#endif

    /* update filter state with measurement */
    hff_update_x(&hff, pos_ned->x, Rgps_pos);
    hff_update_y(&hff, pos_ned->y, Rgps_pos);
#if HFF_UPDATE_GPS_SPEED
    hff_update_xdot(&hff, speed_ned->x, Rgps_vel);
    hff_update_ydot(&hff, speed_ned->y, Rgps_vel);
#endif


#ifdef GPS_LAG
  } else if (hff_rb_n > 0) {
    /* roll back if state was saved approx when GPS was valid */
    lag_counter_err = hff_rb_last->lag_counter - GPS_LAG_N;
    PRINT_DBG(2, ("update. rb_n: %d  lag_counter: %d  lag_cnt_err: %d\n", hff_rb_n, hff_rb_last->lag_counter,
                  lag_counter_err));
    if (abs(lag_counter_err) <= GPS_LAG_TOL_N) {
      hff_rb_last->rollback = true;
      hff_update_x(hff_rb_last, pos_ned->x, Rgps_pos);
      hff_update_y(hff_rb_last, pos_ned->y, Rgps_pos);
#if HFF_UPDATE_GPS_SPEED
      hff_update_xdot(hff_rb_last, speed_ned->x, Rgps_vel);
      hff_update_ydot(hff_rb_last, speed_ned->y, Rgps_vel);
#endif
      past_save_counter = GPS_DT_N - 1; // + lag_counter_err;
      PRINT_DBG(2, ("gps updated. past_save_counter: %d\n", past_save_counter));
      hff_propagate_past(hff_rb_last);
    } else if (lag_counter_err >= GPS_DT_N - (GPS_LAG_TOL_N + 1)) {
      /* apparently missed a GPS update, try next saved state */
      PRINT_DBG(2, ("try next saved state\n"));
      hff_rb_drop_last();
      hff_update_gps(pos_ned, speed_ned);
    }
  } else if (save_counter < 0) {
    /* ringbuffer empty -> save output filter state at next GPS validity point in time */
    save_counter = GPS_DT_N - 1 - (GPS_LAG_N % GPS_DT_N);
    PRINT_DBG(2, ("rb empty, save counter set: %d\n", save_counter));
  }
#endif /* GPS_LAG */
}


void hff_realign(struct FloatVect2 pos, struct FloatVect2 vel)
{
  hff.x = pos.x;
  hff.y = pos.y;
  hff.xdot = vel.x;
  hff.ydot = vel.y;
  hff.xbias = 0.f;
  hff.ybias = 0.f;
#ifdef GPS_LAG
  while (hff_rb_n > 0) {
    hff_rb_drop_last();
  }
  save_counter = -1;
  past_save_counter = SAVE_DONE;
#endif
}


/**
 * Propagate the filter in time.
 *
 * F = [ 1 dt -dt^2/2
 *       0  1 -dt
 *       0  0   1     ];
 *
 * B = [ dt^2/2 dt 0]';
 *
 * Q = [ HFF_Q     0       0
 *       0     HFF_Qdotdot 0
 *       0     0       HFF_Qbiasbias ];
 *
 * Xk1 = F * Xk0 + B * accel;
 *
 * Pk1 = F * Pk0 * F' + Q;
 *
 */
static void hff_propagate_x(struct HfilterFloat *filt, float dt)
{
  /* update state */
  filt->xdotdot = hff_xdd_meas - filt->xbias;
  filt->x = filt->x + filt->xdot * dt;// + filt->xdotdot * dt * dt / 2;
  filt->xdot = filt->xdot + dt * filt->xdotdot;
  /* update covariance */
  const float FPF00 = filt->xP[0][0] + dt * (filt->xP[1][0] + filt->xP[0][1] + dt * filt->xP[1][1]);
  const float FPF01 = filt->xP[0][1] + dt * (filt->xP[1][1] - filt->xP[0][2] - dt * filt->xP[1][2]);
  const float FPF02 = filt->xP[0][2] + dt * (filt->xP[1][2]);
  const float FPF10 = filt->xP[1][0] + dt * (-filt->xP[2][0] + filt->xP[1][1] - dt * filt->xP[2][1]);
  const float FPF11 = filt->xP[1][1] + dt * (-filt->xP[2][1] - filt->xP[1][2] + dt * filt->xP[2][2]);
  const float FPF12 = filt->xP[1][2] + dt * (-filt->xP[2][2]);
  const float FPF20 = filt->xP[2][0] + dt * (filt->xP[2][1]);
  const float FPF21 = filt->xP[2][1] + dt * (-filt->xP[2][2]);
  const float FPF22 = filt->xP[2][2];

  filt->xP[0][0] = FPF00 + HFF_Q * dt * dt / 2.;
  filt->xP[0][1] = FPF01;
  filt->xP[0][2] = FPF02;
  filt->xP[1][0] = FPF10;
  filt->xP[1][1] = FPF11 + HFF_Qdotdot * dt;
  filt->xP[1][2] = FPF12;
  filt->xP[2][0] = FPF20;
  filt->xP[2][1] = FPF21;
  filt->xP[2][2] = FPF22 + HFF_Qbiasbias;
}

static void hff_propagate_y(struct HfilterFloat *filt, float dt)
{
  /* update state */
  filt->ydotdot = hff_ydd_meas - filt->ybias;
  filt->y = filt->y + dt * filt->ydot;// + filt->ydotdot * dt * dt / 2;
  filt->ydot = filt->ydot + dt * filt->ydotdot;
  /* update covariance */
  const float FPF00 = filt->yP[0][0] + dt * (filt->yP[1][0] + filt->yP[0][1] + dt * filt->yP[1][1]);
  const float FPF01 = filt->yP[0][1] + dt * (filt->yP[1][1] - filt->yP[0][2] - dt * filt->yP[1][2]);
  const float FPF02 = filt->yP[0][2] + dt * (filt->yP[1][2]);
  const float FPF10 = filt->yP[1][0] + dt * (-filt->yP[2][0] + filt->yP[1][1] - dt * filt->yP[2][1]);
  const float FPF11 = filt->yP[1][1] + dt * (-filt->yP[2][1] - filt->yP[1][2] + dt * filt->yP[2][2]);
  const float FPF12 = filt->yP[1][2] + dt * (-filt->yP[2][2]);
  const float FPF20 = filt->yP[2][0] + dt * (filt->yP[2][1]);
  const float FPF21 = filt->yP[2][1] + dt * (-filt->yP[2][2]);
  const float FPF22 = filt->yP[2][2];

  filt->yP[0][0] = FPF00 + HFF_Q * dt * dt / 2.;
  filt->yP[0][1] = FPF01;
  filt->yP[0][2] = FPF02;
  filt->yP[1][0] = FPF10;
  filt->yP[1][1] = FPF11 + HFF_Qdotdot * dt;
  filt->yP[1][2] = FPF12;
  filt->yP[2][0] = FPF20;
  filt->yP[2][1] = FPF21;
  filt->yP[2][2] = FPF22 + HFF_Qbiasbias;
}


/**
 * Update position
 *
 * H = [1 0 0];
 * R = 0.1;
 * // state residual
 * y = rangemeter - H * Xm;
 * // covariance residual
 * S = H*Pm*H' + R;
 * // kalman gain
 * K = Pm*H'*inv(S);
 * // update state
 * Xp = Xm + K*y;
 * // update covariance
 * Pp = Pm - K*H*Pm;
 */
void hff_update_pos(struct FloatVect2 pos, struct FloatVect2 Rpos)
{
  hff_lost_counter = 0;
  hff_update_x(&hff, pos.x, Rpos.x);
  hff_update_y(&hff, pos.y, Rpos.y);
}

static void hff_update_x(struct HfilterFloat *filt, float x_meas, float Rpos)
{
  hff_x_meas = x_meas;

  const float y  = x_meas - filt->x;
  const float S  = filt->xP[0][0] + Rpos;
  const float K1 = filt->xP[0][0] * 1 / S;
  const float K2 = filt->xP[1][0] * 1 / S;
  const float K3 = filt->xP[2][0] * 1 / S;

  filt->x     = filt->x     + K1 * y;
  filt->xdot  = filt->xdot  + K2 * y;
  filt->xbias = filt->xbias + K3 * y;

  const float P11 = (1. - K1) * filt->xP[0][0];
  const float P12 = (1. - K1) * filt->xP[0][1];
  const float P13 = (1. - K1) * filt->xP[0][2];
  const float P21 = -K2 * filt->xP[0][0] + filt->xP[1][0];
  const float P22 = -K2 * filt->xP[0][1] + filt->xP[1][1];
  const float P23 = -K2 * filt->xP[0][2] + filt->xP[1][2];
  const float P31 = -K3 * filt->xP[0][0] + filt->xP[2][0];
  const float P32 = -K3 * filt->xP[0][1] + filt->xP[2][1];
  const float P33 = -K3 * filt->xP[0][2] + filt->xP[2][2];

  filt->xP[0][0] = P11;
  filt->xP[0][1] = P12;
  filt->xP[0][2] = P13;
  filt->xP[1][0] = P21;
  filt->xP[1][1] = P22;
  filt->xP[1][2] = P23;
  filt->xP[2][0] = P31;
  filt->xP[2][1] = P32;
  filt->xP[2][2] = P33;
}

static void hff_update_y(struct HfilterFloat *filt, float y_meas, float Rpos)
{
  hff_y_meas = y_meas;

  const float y  = y_meas - filt->y;
  const float S  = filt->yP[0][0] + Rpos;
  const float K1 = filt->yP[0][0] * 1 / S;
  const float K2 = filt->yP[1][0] * 1 / S;
  const float K3 = filt->yP[2][0] * 1 / S;

  filt->y     = filt->y     + K1 * y;
  filt->ydot  = filt->ydot  + K2 * y;
  filt->ybias = filt->ybias + K3 * y;

  const float P11 = (1. - K1) * filt->yP[0][0];
  const float P12 = (1. - K1) * filt->yP[0][1];
  const float P13 = (1. - K1) * filt->yP[0][2];
  const float P21 = -K2 * filt->yP[0][0] + filt->yP[1][0];
  const float P22 = -K2 * filt->yP[0][1] + filt->yP[1][1];
  const float P23 = -K2 * filt->yP[0][2] + filt->yP[1][2];
  const float P31 = -K3 * filt->yP[0][0] + filt->yP[2][0];
  const float P32 = -K3 * filt->yP[0][1] + filt->yP[2][1];
  const float P33 = -K3 * filt->yP[0][2] + filt->yP[2][2];

  filt->yP[0][0] = P11;
  filt->yP[0][1] = P12;
  filt->yP[0][2] = P13;
  filt->yP[1][0] = P21;
  filt->yP[1][1] = P22;
  filt->yP[1][2] = P23;
  filt->yP[2][0] = P31;
  filt->yP[2][1] = P32;
  filt->yP[2][2] = P33;
}


/*
 *
 * Update velocity
 *
 * H = [0 1 0];
 * R = 0.1;
 * // state residual
 * yd = vx - H * Xm;
 * // covariance residual
 * S = H*Pm*H' + R;
 * // kalman gain
 * K = Pm*H'*inv(S);
 * // update state
 * Xp = Xm + K*yd;
 * // update covariance
 * Pp = Pm - K*H*Pm;
*/
void hff_update_vel(struct FloatVect2 vel, struct FloatVect2 Rvel)
{
  if (Rvel.x >= 0.f) {
    hff_update_xdot(&hff, vel.x, Rvel.x);
  }
  if (Rvel.y >= 0.f) {
    hff_update_ydot(&hff, vel.y, Rvel.y);
  }

  if (Rvel.x >= 0.f || Rvel.y >= 0.f) {
    hff_speed_lost_counter = 0;
  }
}

static void hff_update_xdot(struct HfilterFloat *filt, float vel, float Rvel)
{
  hff_xd_meas = vel;

  const float yd = vel - filt->xdot;
  const float S  = filt->xP[1][1] + Rvel;
  const float K1 = filt->xP[0][1] * 1 / S;
  const float K2 = filt->xP[1][1] * 1 / S;
  const float K3 = filt->xP[2][1] * 1 / S;

  filt->x     = filt->x     + K1 * yd;
  filt->xdot  = filt->xdot  + K2 * yd;
  filt->xbias = filt->xbias + K3 * yd;

  const float P11 = -K1 * filt->xP[1][0] + filt->xP[0][0];
  const float P12 = -K1 * filt->xP[1][1] + filt->xP[0][1];
  const float P13 = -K1 * filt->xP[1][2] + filt->xP[0][2];
  const float P21 = (1. - K2) * filt->xP[1][0];
  const float P22 = (1. - K2) * filt->xP[1][1];
  const float P23 = (1. - K2) * filt->xP[1][2];
  const float P31 = -K3 * filt->xP[1][0] + filt->xP[2][0];
  const float P32 = -K3 * filt->xP[1][1] + filt->xP[2][1];
  const float P33 = -K3 * filt->xP[1][2] + filt->xP[2][2];

  filt->xP[0][0] = P11;
  filt->xP[0][1] = P12;
  filt->xP[0][2] = P13;
  filt->xP[1][0] = P21;
  filt->xP[1][1] = P22;
  filt->xP[1][2] = P23;
  filt->xP[2][0] = P31;
  filt->xP[2][1] = P32;
  filt->xP[2][2] = P33;
}

static void hff_update_ydot(struct HfilterFloat *filt, float vel, float Rvel)
{
  hff_yd_meas = vel;

  const float yd = vel - filt->ydot;
  const float S  = filt->yP[1][1] + Rvel;
  const float K1 = filt->yP[0][1] * 1 / S;
  const float K2 = filt->yP[1][1] * 1 / S;
  const float K3 = filt->yP[2][1] * 1 / S;

  filt->y     = filt->y     + K1 * yd;
  filt->ydot  = filt->ydot  + K2 * yd;
  filt->ybias = filt->ybias + K3 * yd;

  const float P11 = -K1 * filt->yP[1][0] + filt->yP[0][0];
  const float P12 = -K1 * filt->yP[1][1] + filt->yP[0][1];
  const float P13 = -K1 * filt->yP[1][2] + filt->yP[0][2];
  const float P21 = (1. - K2) * filt->yP[1][0];
  const float P22 = (1. - K2) * filt->yP[1][1];
  const float P23 = (1. - K2) * filt->yP[1][2];
  const float P31 = -K3 * filt->yP[1][0] + filt->yP[2][0];
  const float P32 = -K3 * filt->yP[1][1] + filt->yP[2][1];
  const float P33 = -K3 * filt->yP[1][2] + filt->yP[2][2];

  filt->yP[0][0] = P11;
  filt->yP[0][1] = P12;
  filt->yP[0][2] = P13;
  filt->yP[1][0] = P21;
  filt->yP[1][1] = P22;
  filt->yP[1][2] = P23;
  filt->yP[2][0] = P31;
  filt->yP[2][1] = P32;
  filt->yP[2][2] = P33;
}
