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

/*

X_x = [ x xdot xbias ]
X_y = [ y ydot ybias ]


*/

/* horizontal filter propagation frequency */
#define HFF_FREQ 512./HFF_PRESCALER
#define DT_HFILTER 1./HFF_FREQ

/* initial covariance diagonal */
#define INIT_PXX 1.
/* process noise (is the same for x and y)*/
#define ACCEL_NOISE 0.5
#define Q       ACCEL_NOISE*DT_HFILTER*DT_HFILTER/2.
#define Qdotdot ACCEL_NOISE*DT_HFILTER
#define Qbiasbias 1e-7*HFF_PRESCALER
//TODO: proper measurement noise
#define Rpos   1.
#define Rspeed 1.

float b2_hff_x;
float b2_hff_xbias;
float b2_hff_xdot;
float b2_hff_xdotdot;

float b2_hff_y;
float b2_hff_ybias;
float b2_hff_ydot;
float b2_hff_ydotdot;

float b2_hff_xP[B2_HFF_STATE_SIZE][B2_HFF_STATE_SIZE];
float b2_hff_yP[B2_HFF_STATE_SIZE][B2_HFF_STATE_SIZE];

float b2_hff_x_meas;
float b2_hff_y_meas;


static inline void b2_hff_init_x(float init_x, float init_xdot, float init_xbias);
static inline void b2_hff_init_y(float init_y, float init_ydot, float init_ybias);

static inline void b2_hff_propagate_x(float xaccel);
static inline void b2_hff_propagate_y(float yaccel);

static inline void b2_hff_update_x(float x_meas);
static inline void b2_hff_update_y(float y_meas);

static inline void b2_hff_update_xdot(float v);
static inline void b2_hff_update_ydot(float v);



void b2_hff_init(float init_x, float init_xdot, float init_xbias, float init_y, float init_ydot, float init_ybias) {
    b2_hff_init_x(init_x, init_xdot, init_xbias);
    b2_hff_init_y(init_y, init_ydot, init_ybias);
}

static inline void b2_hff_init_x(float init_x, float init_xdot, float init_xbias) {
  b2_hff_x     = init_x;
  b2_hff_xdot  = init_xdot;
  b2_hff_xbias = init_xbias;
  int i, j;
  for (i=0; i<B2_HFF_STATE_SIZE; i++) {
    for (j=0; j<B2_HFF_STATE_SIZE; j++)
      b2_hff_xP[i][j] = 0.;
    b2_hff_xP[i][i] = INIT_PXX;
  }

}

static inline void b2_hff_init_y(float init_y, float init_ydot, float init_ybias) {
  b2_hff_y     = init_y;
  b2_hff_ydot  = init_ydot;
  b2_hff_ybias = init_ybias;
  int i, j;
  for (i=0; i<B2_HFF_STATE_SIZE; i++) {
    for (j=0; j<B2_HFF_STATE_SIZE; j++)
      b2_hff_yP[i][j] = 0.;
    b2_hff_yP[i][i] = INIT_PXX;
  }

}


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
    b2_hff_propagate_x(xaccel);
    b2_hff_propagate_y(yaccel);
}

static inline void b2_hff_propagate_x(float xaccel) {
  /* update state */
  b2_hff_xdotdot = xaccel - b2_hff_xbias;
  b2_hff_x = b2_hff_x + DT_HFILTER * b2_hff_xdot;
  b2_hff_xdot = b2_hff_xdot + DT_HFILTER * b2_hff_xdotdot;
  /* update covariance */
  const float FPF00 = b2_hff_xP[0][0] + DT_HFILTER * ( b2_hff_xP[1][0] + b2_hff_xP[0][1] + DT_HFILTER * b2_hff_xP[1][1] );
  const float FPF01 = b2_hff_xP[0][1] + DT_HFILTER * ( b2_hff_xP[1][1] - b2_hff_xP[0][2] - DT_HFILTER * b2_hff_xP[1][2] );
  const float FPF02 = b2_hff_xP[0][2] + DT_HFILTER * ( b2_hff_xP[1][2] );
  const float FPF10 = b2_hff_xP[1][0] + DT_HFILTER * (-b2_hff_xP[2][0] + b2_hff_xP[1][1] - DT_HFILTER * b2_hff_xP[2][1] );
  const float FPF11 = b2_hff_xP[1][1] + DT_HFILTER * (-b2_hff_xP[2][1] - b2_hff_xP[1][2] + DT_HFILTER * b2_hff_xP[2][2] );
  const float FPF12 = b2_hff_xP[1][2] + DT_HFILTER * (-b2_hff_xP[2][2] );
  const float FPF20 = b2_hff_xP[2][0] + DT_HFILTER * ( b2_hff_xP[2][1] );
  const float FPF21 = b2_hff_xP[2][1] + DT_HFILTER * (-b2_hff_xP[2][2] );
  const float FPF22 = b2_hff_xP[2][2];

  b2_hff_xP[0][0] = FPF00 + Q;
  b2_hff_xP[0][1] = FPF01;
  b2_hff_xP[0][2] = FPF02;
  b2_hff_xP[1][0] = FPF10;
  b2_hff_xP[1][1] = FPF11 + Qdotdot;
  b2_hff_xP[1][2] = FPF12;
  b2_hff_xP[2][0] = FPF20;
  b2_hff_xP[2][1] = FPF21;
  b2_hff_xP[2][2] = FPF22 + Qbiasbias;

}

static inline void b2_hff_propagate_y(float yaccel) {
  /* update state */
  b2_hff_ydotdot = yaccel - b2_hff_ybias;
  b2_hff_y = b2_hff_y + DT_HFILTER * b2_hff_ydot;
  b2_hff_ydot = b2_hff_ydot + DT_HFILTER * b2_hff_ydotdot;
  /* update covariance */
  const float FPF00 = b2_hff_yP[0][0] + DT_HFILTER * ( b2_hff_yP[1][0] + b2_hff_yP[0][1] + DT_HFILTER * b2_hff_yP[1][1] );
  const float FPF01 = b2_hff_yP[0][1] + DT_HFILTER * ( b2_hff_yP[1][1] - b2_hff_yP[0][2] - DT_HFILTER * b2_hff_yP[1][2] );
  const float FPF02 = b2_hff_yP[0][2] + DT_HFILTER * ( b2_hff_yP[1][2] );
  const float FPF10 = b2_hff_yP[1][0] + DT_HFILTER * (-b2_hff_yP[2][0] + b2_hff_yP[1][1] - DT_HFILTER * b2_hff_yP[2][1] );
  const float FPF11 = b2_hff_yP[1][1] + DT_HFILTER * (-b2_hff_yP[2][1] - b2_hff_yP[1][2] + DT_HFILTER * b2_hff_yP[2][2] );
  const float FPF12 = b2_hff_yP[1][2] + DT_HFILTER * (-b2_hff_yP[2][2] );
  const float FPF20 = b2_hff_yP[2][0] + DT_HFILTER * ( b2_hff_yP[2][1] );
  const float FPF21 = b2_hff_yP[2][1] + DT_HFILTER * (-b2_hff_yP[2][2] );
  const float FPF22 = b2_hff_yP[2][2];

  b2_hff_yP[0][0] = FPF00 + Q;
  b2_hff_yP[0][1] = FPF01;
  b2_hff_yP[0][2] = FPF02;
  b2_hff_yP[1][0] = FPF10;
  b2_hff_yP[1][1] = FPF11 + Qdotdot;
  b2_hff_yP[1][2] = FPF12;
  b2_hff_yP[2][0] = FPF20;
  b2_hff_yP[2][1] = FPF21;
  b2_hff_yP[2][2] = FPF22 + Qbiasbias;

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

  const float y = x_meas - b2_hff_x;
  const float S = b2_hff_xP[0][0] + Rpos;
  const float K1 = b2_hff_xP[0][0] * 1/S;
  const float K2 = b2_hff_xP[1][0] * 1/S;
  const float K3 = b2_hff_xP[2][0] * 1/S;

  b2_hff_x     = b2_hff_x     + K1 * y;
  b2_hff_xdot  = b2_hff_xdot  + K2 * y;
  b2_hff_xbias = b2_hff_xbias + K3 * y;

  const float P11 = (1. - K1) * b2_hff_xP[0][0];
  const float P12 = (1. - K1) * b2_hff_xP[0][1];
  const float P13 = (1. - K1) * b2_hff_xP[0][2];
  const float P21 = -K2 * b2_hff_xP[0][0] + b2_hff_xP[1][0];
  const float P22 = -K2 * b2_hff_xP[0][1] + b2_hff_xP[1][1];
  const float P23 = -K2 * b2_hff_xP[0][2] + b2_hff_xP[1][2];
  const float P31 = -K3 * b2_hff_xP[0][0] + b2_hff_xP[2][0];
  const float P32 = -K3 * b2_hff_xP[0][1] + b2_hff_xP[2][1];
  const float P33 = -K3 * b2_hff_xP[0][2] + b2_hff_xP[2][2];

  b2_hff_xP[0][0] = P11;
  b2_hff_xP[0][1] = P12;
  b2_hff_xP[0][2] = P13;
  b2_hff_xP[1][0] = P21;
  b2_hff_xP[1][1] = P22;
  b2_hff_xP[1][2] = P23;
  b2_hff_xP[2][0] = P31;
  b2_hff_xP[2][1] = P32;
  b2_hff_xP[2][2] = P33;

}

static inline void b2_hff_update_y(float y_meas) {
  b2_hff_y_meas = y_meas;

  const float y = y_meas - b2_hff_y;
  const float S = b2_hff_yP[0][0] + Rpos;
  const float K1 = b2_hff_yP[0][0] * 1/S;
  const float K2 = b2_hff_yP[1][0] * 1/S;
  const float K3 = b2_hff_yP[2][0] * 1/S;

  b2_hff_y     = b2_hff_y     + K1 * y;
  b2_hff_ydot  = b2_hff_ydot  + K2 * y;
  b2_hff_ybias = b2_hff_ybias + K3 * y;

  const float P11 = (1. - K1) * b2_hff_yP[0][0];
  const float P12 = (1. - K1) * b2_hff_yP[0][1];
  const float P13 = (1. - K1) * b2_hff_yP[0][2];
  const float P21 = -K2 * b2_hff_yP[0][0] + b2_hff_yP[1][0];
  const float P22 = -K2 * b2_hff_yP[0][1] + b2_hff_yP[1][1];
  const float P23 = -K2 * b2_hff_yP[0][2] + b2_hff_yP[1][2];
  const float P31 = -K3 * b2_hff_yP[0][0] + b2_hff_yP[2][0];
  const float P32 = -K3 * b2_hff_yP[0][1] + b2_hff_yP[2][1];
  const float P33 = -K3 * b2_hff_yP[0][2] + b2_hff_yP[2][2];

  b2_hff_yP[0][0] = P11;
  b2_hff_yP[0][1] = P12;
  b2_hff_yP[0][2] = P13;
  b2_hff_yP[1][0] = P21;
  b2_hff_yP[1][1] = P22;
  b2_hff_yP[1][2] = P23;
  b2_hff_yP[2][0] = P31;
  b2_hff_yP[2][1] = P32;
  b2_hff_yP[2][2] = P33;

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
  const float yd = v - b2_hff_xdot;
  const float S = b2_hff_xP[1][1] + Rspeed;
  const float K1 = b2_hff_xP[0][1] * 1/S;
  const float K2 = b2_hff_xP[1][1] * 1/S;
  const float K3 = b2_hff_xP[2][1] * 1/S;

  b2_hff_x     = b2_hff_x     + K1 * yd;
  b2_hff_xdot  = b2_hff_xdot  + K2 * yd;
  b2_hff_xbias = b2_hff_xbias + K3 * yd;

  const float P11 = -K1 * b2_hff_xP[1][0] + b2_hff_xP[0][0];
  const float P12 = -K1 * b2_hff_xP[1][1] + b2_hff_xP[0][1];
  const float P13 = -K1 * b2_hff_xP[1][2] + b2_hff_xP[0][2];
  const float P21 = (1. - K2) * b2_hff_xP[1][0];
  const float P22 = (1. - K2) * b2_hff_xP[1][1];
  const float P23 = (1. - K2) * b2_hff_xP[1][2];
  const float P31 = -K3 * b2_hff_xP[1][0] + b2_hff_xP[2][0];
  const float P32 = -K3 * b2_hff_xP[1][1] + b2_hff_xP[2][1];
  const float P33 = -K3 * b2_hff_xP[1][2] + b2_hff_xP[2][2];

  b2_hff_xP[0][0] = P11;
  b2_hff_xP[0][1] = P12;
  b2_hff_xP[0][2] = P13;
  b2_hff_xP[1][0] = P21;
  b2_hff_xP[1][1] = P22;
  b2_hff_xP[1][2] = P23;
  b2_hff_xP[2][0] = P31;
  b2_hff_xP[2][1] = P32;
  b2_hff_xP[2][2] = P33;

}

static inline void b2_hff_update_ydot(float v) {
  const float yd = v - b2_hff_ydot;
  const float S = b2_hff_yP[1][1] + Rspeed;
  const float K1 = b2_hff_yP[0][1] * 1/S;
  const float K2 = b2_hff_yP[1][1] * 1/S;
  const float K3 = b2_hff_yP[2][1] * 1/S;

  b2_hff_y     = b2_hff_y     + K1 * yd;
  b2_hff_ydot  = b2_hff_ydot  + K2 * yd;
  b2_hff_ybias = b2_hff_ybias + K3 * yd;

  const float P11 = -K1 * b2_hff_yP[1][0] + b2_hff_yP[0][0];
  const float P12 = -K1 * b2_hff_yP[1][1] + b2_hff_yP[0][1];
  const float P13 = -K1 * b2_hff_yP[1][2] + b2_hff_yP[0][2];
  const float P21 = (1. - K2) * b2_hff_yP[1][0];
  const float P22 = (1. - K2) * b2_hff_yP[1][1];
  const float P23 = (1. - K2) * b2_hff_yP[1][2];
  const float P31 = -K3 * b2_hff_yP[1][0] + b2_hff_yP[2][0];
  const float P32 = -K3 * b2_hff_yP[1][1] + b2_hff_yP[2][1];
  const float P33 = -K3 * b2_hff_yP[1][2] + b2_hff_yP[2][2];

  b2_hff_yP[0][0] = P11;
  b2_hff_yP[0][1] = P12;
  b2_hff_yP[0][2] = P13;
  b2_hff_yP[1][0] = P21;
  b2_hff_yP[1][1] = P22;
  b2_hff_yP[1][2] = P23;
  b2_hff_yP[2][0] = P31;
  b2_hff_yP[2][1] = P32;
  b2_hff_yP[2][2] = P33;

}


