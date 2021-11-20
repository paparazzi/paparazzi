/*
 * Copyright (C) 2021 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file "modules/meteo/wind_estimation_quadrotor.c"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Wind estimation from quadrotor motion
 */

#include "modules/meteo/wind_estimation_quadrotor.h"
#include "filters/linear_kalman_filter.h"
#include "math/pprz_isa.h"
#include "state.h"
#include "generated/airframe.h"
#include "modules/datalink/downlink.h"
#include "generated/modules.h"
#include "modules/core/abi.h"
#include "modules/air_data/air_data.h"

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
#endif

#define WE_QUAD_STATUS_IDLE 0
#define WE_QUAD_STATUS_RUN  1

#define WE_QUAD_STATE_SIZE  4
#define WE_QUAD_CMD_SIZE    2
#define WE_QUAD_MEAS_SIZE   2

#define WE_QUAD_VA_X  0
#define WE_QUAD_W_X   1
#define WE_QUAD_VA_Y  2
#define WE_QUAD_W_Y   3

#ifndef WE_QUAD_P0_VA
#define WE_QUAD_P0_VA 1.f
#endif

#ifndef WE_QUAD_P0_W
#define WE_QUAD_P0_W  1.f
#endif

#ifndef WE_QUAD_Q_VA
#define WE_QUAD_Q_VA  .05f
#endif

#ifndef WE_QUAD_Q_W
#define WE_QUAD_Q_W   .001f
#endif

#ifndef WE_QUAD_R
#define WE_QUAD_R     .5f
#endif

// update wind in state interface directly
#ifndef WE_QUAD_UPDATE_STATE
#define WE_QUAD_UPDATE_STATE TRUE
#endif

static const float we_dt = WIND_ESTIMATION_QUADROTOR_PERIODIC_PERIOD;

static void send_wind(struct transport_tx *trans, struct link_device *dev);

struct wind_estimation_quadrotor {
  struct linear_kalman_filter filter;
  uint8_t status;
};

static struct wind_estimation_quadrotor we_quad;
struct wind_estimation_quadrotor_params we_quad_params;

static void wind_estimation_quadrotor_reset(void)
{
  we_quad.filter.P[0][0] = WE_QUAD_P0_VA;
  we_quad.filter.P[1][1] = WE_QUAD_P0_W;
  we_quad.filter.P[2][2] = WE_QUAD_P0_VA;
  we_quad.filter.P[3][3] = WE_QUAD_P0_W;
  float_vect_zero(we_quad.filter.X, WE_QUAD_STATE_SIZE);
}

void wind_estimation_quadrotor_init(void)
{
  we_quad.status = WE_QUAD_STATUS_IDLE;

  // init filter and model
  linear_kalman_filter_init(&we_quad.filter, WE_QUAD_STATE_SIZE, WE_QUAD_CMD_SIZE, WE_QUAD_MEAS_SIZE);
  we_quad.filter.A[0][0] = 1.f - WE_QUAD_DRAG * we_dt / WE_QUAD_MASS;
  we_quad.filter.A[1][1] = 1.f;
  we_quad.filter.A[2][2] = 1.f - WE_QUAD_DRAG * we_dt / WE_QUAD_MASS;
  we_quad.filter.A[3][3] = 1.f;
  we_quad.filter.B[0][0] = we_dt / WE_QUAD_MASS;
  we_quad.filter.B[2][1] = we_dt / WE_QUAD_MASS;
  we_quad.filter.C[0][0] = 1.f;
  we_quad.filter.C[0][1] = 1.f;
  we_quad.filter.C[1][2] = 1.f;
  we_quad.filter.C[1][3] = 1.f;
  we_quad.filter.Q[0][0] = WE_QUAD_Q_VA;
  we_quad.filter.Q[1][1] = WE_QUAD_Q_W;
  we_quad.filter.Q[2][2] = WE_QUAD_Q_VA;
  we_quad.filter.Q[3][3] = WE_QUAD_Q_W;
  we_quad.filter.R[0][0] = WE_QUAD_R;
  we_quad.filter.R[1][1] = WE_QUAD_R;
  // reset covariance and state
  wind_estimation_quadrotor_reset();
  // external params
  we_quad_params.Q_va = WE_QUAD_Q_VA;
  we_quad_params.Q_w = WE_QUAD_Q_W;
  we_quad_params.R = WE_QUAD_R;

#if PERIODIC_TELEMETRY
  // register for periodic telemetry
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WIND_INFO_RET, send_wind);
#endif
}

void wind_estimation_quadrotor_periodic(void)
{
  // compute command from state interface
  float U[WE_QUAD_CMD_SIZE];
  struct FloatRMat rot = *stateGetNedToBodyRMat_f();
  float coef = Max(MAT33_ELMT(rot, 2, 2), 0.5); // limit to 1/2
  float Tnorm = WE_QUAD_MASS * PPRZ_ISA_GRAVITY / coef;
  struct FloatVect3 Tbody = { 0.f, 0.f, -Tnorm };
  struct FloatVect3 Tned;
  float_rmat_transp_vmult(&Tned, &rot, &Tbody);
  U[0] = Tned.x;
  U[1] = Tned.y;

  linear_kalman_filter_predict(&we_quad.filter, U);

  // compute correction from measure
  float Y[WE_QUAD_MEAS_SIZE];
  Y[0] = stateGetSpeedNed_f()->x; // north ground speed
  Y[1] = stateGetSpeedNed_f()->y; // east ground speed

  linear_kalman_filter_update(&we_quad.filter,Y);

  // send airspeed as ABI message
  struct FloatVect2 tas = { we_quad.filter.X[WE_QUAD_VA_X], we_quad.filter.X[WE_QUAD_VA_Y] };
  float eas = eas_from_tas(float_vect2_norm(&tas));
  AbiSendMsgAIRSPEED(AIRSPEED_WE_QUAD_ID, eas);
  // update wind in state interface only if enabled
#if WE_QUAD_UPDATE_STATE
  struct FloatVect2 wind_ne = {
    we_quad.filter.X[WE_QUAD_W_X],
    we_quad.filter.X[WE_QUAD_W_Y]
  };
  stateSetHorizontalWindspeed_f(&wind_ne);
#endif
}

void wind_estimation_quadrotor_stop(void)
{
  we_quad.status = WE_QUAD_STATUS_IDLE;
#if WE_QUAD_UPDATE_STATE
  struct FloatVect2 zero_wind_ne = { 0.f, 0.f };
  stateSetHorizontalWindspeed_f(&zero_wind_ne);
#endif
}

void wind_estimation_quadrotor_start(void)
{
  wind_estimation_quadrotor_reset();
  we_quad.status = WE_QUAD_STATUS_RUN;
}

float wind_estimation_quadrotor_SetQva(float Q_va)
{
  we_quad_params.Q_va = Q_va;
  we_quad.filter.Q[0][0] = Q_va;
  we_quad.filter.Q[2][2] = Q_va;
  return Q_va;
}

float wind_estimation_quadrotor_SetQw(float Q_w)
{
  we_quad_params.Q_w = Q_w;
  we_quad.filter.Q[1][1] = Q_w;
  we_quad.filter.Q[3][3] = Q_w;
  return Q_w;
}

float wind_estimation_quadrotor_SetR(float R)
{
  we_quad_params.R = R;
  we_quad.filter.R[0][0] = R;
  we_quad.filter.R[1][1] = R;
  return R;
}

void wind_estimation_quadrotor_report(void)
{
  DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, WE_QUAD_STATE_SIZE, we_quad.filter.X);
}

static void send_wind(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t flags = 5; // send horizontal wind and airspeed
  float upwind = 0.f;
  struct FloatVect2 as = { we_quad.filter.X[WE_QUAD_VA_X], we_quad.filter.X[WE_QUAD_VA_Y] };
  float airspeed = float_vect2_norm(&as);
  pprz_msg_send_WIND_INFO_RET(trans, dev, AC_ID,
      &flags,
      &we_quad.filter.X[WE_QUAD_W_Y], // east
      &we_quad.filter.X[WE_QUAD_W_X], // north
      &upwind,
      &airspeed);
}

