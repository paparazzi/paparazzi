/*
 *
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

/** \file stabilization_attitude_quat_int.c
 * \brief Booz quaternion attitude stabilization
 */

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/quat_setpoint.h"

#include <stdio.h>
#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "subsystems/ahrs.h"
#include "generated/airframe.h"

struct Int32AttitudeGains stabilization_gains = {
  {STABILIZATION_ATTITUDE_PHI_PGAIN, STABILIZATION_ATTITUDE_THETA_PGAIN, STABILIZATION_ATTITUDE_PSI_PGAIN },
  {STABILIZATION_ATTITUDE_PHI_DGAIN, STABILIZATION_ATTITUDE_THETA_DGAIN, STABILIZATION_ATTITUDE_PSI_DGAIN },
  {STABILIZATION_ATTITUDE_PHI_DDGAIN, STABILIZATION_ATTITUDE_THETA_DDGAIN, STABILIZATION_ATTITUDE_PSI_DDGAIN },
  {STABILIZATION_ATTITUDE_PHI_IGAIN, STABILIZATION_ATTITUDE_THETA_IGAIN, STABILIZATION_ATTITUDE_PSI_IGAIN }
};

struct Int32Quat stabilization_att_sum_err_quat;
struct Int32Eulers stabilization_att_sum_err;

int32_t stabilization_att_fb_cmd[COMMANDS_NB];
int32_t stabilization_att_ff_cmd[COMMANDS_NB];

//static int gain_idx = STABILIZATION_ATTITUDE_GAIN_IDX_DEFAULT;

/*
static const float phi_pgain[] = STABILIZATION_ATTITUDE_PHI_PGAIN;
static const float theta_pgain[] = STABILIZATION_ATTITUDE_THETA_PGAIN;
static const float psi_pgain[] = STABILIZATION_ATTITUDE_PSI_PGAIN;

static const float phi_dgain[] = STABILIZATION_ATTITUDE_PHI_DGAIN;
static const float theta_dgain[] = STABILIZATION_ATTITUDE_THETA_DGAIN;
static const float psi_dgain[] = STABILIZATION_ATTITUDE_PSI_DGAIN;

static const float phi_igain[] = STABILIZATION_ATTITUDE_PHI_IGAIN;
static const float theta_igain[] = STABILIZATION_ATTITUDE_THETA_IGAIN;
static const float psi_igain[] = STABILIZATION_ATTITUDE_PSI_IGAIN;

static const float phi_ddgain[] = STABILIZATION_ATTITUDE_PHI_DDGAIN;
static const float theta_ddgain[] = STABILIZATION_ATTITUDE_THETA_DDGAIN;
static const float psi_ddgain[] = STABILIZATION_ATTITUDE_PSI_DDGAIN;

static const float phi_dgain_d[] = STABILIZATION_ATTITUDE_PHI_DGAIN_D;
static const float theta_dgain_d[] = STABILIZATION_ATTITUDE_THETA_DGAIN_D;
static const float psi_dgain_d[] = STABILIZATION_ATTITUDE_PSI_DGAIN_D;

static const float phi_pgain_surface[] = STABILIZATION_ATTITUDE_PHI_PGAIN_SURFACE;
static const float theta_pgain_surface[] = STABILIZATION_ATTITUDE_THETA_PGAIN_SURFACE;
static const float psi_pgain_surface[] = STABILIZATION_ATTITUDE_PSI_PGAIN_SURFACE;

static const float phi_dgain_surface[] = STABILIZATION_ATTITUDE_PHI_DGAIN_SURFACE;
static const float theta_dgain_surface[] = STABILIZATION_ATTITUDE_THETA_DGAIN_SURFACE;
static const float psi_dgain_surface[] = STABILIZATION_ATTITUDE_PSI_DGAIN_SURFACE;

static const float phi_igain_surface[] = STABILIZATION_ATTITUDE_PHI_IGAIN_SURFACE;
static const float theta_igain_surface[] = STABILIZATION_ATTITUDE_THETA_IGAIN_SURFACE;
static const float psi_igain_surface[] = STABILIZATION_ATTITUDE_PSI_IGAIN_SURFACE;

static const float phi_ddgain_surface[] = STABILIZATION_ATTITUDE_PHI_DDGAIN_SURFACE;
static const float theta_ddgain_surface[] = STABILIZATION_ATTITUDE_THETA_DDGAIN_SURFACE;
static const float psi_ddgain_surface[] = STABILIZATION_ATTITUDE_PSI_DDGAIN_SURFACE;
*/

#define IERROR_SCALE 1024
#define GAIN_PRESCALER_FF 1
#define GAIN_PRESCALER_P 1
#define GAIN_PRESCALER_D 1
#define GAIN_PRESCALER_I 1

void stabilization_attitude_init(void) {

  stabilization_attitude_ref_init();

  /*
  for (int i = 0; i < STABILIZATION_ATTITUDE_GAIN_NB; i++) {
    VECT3_ASSIGN(stabilization_gains[i].p, phi_pgain[i], theta_pgain[i], psi_pgain[i]);
    VECT3_ASSIGN(stabilization_gains[i].d, phi_dgain[i], theta_dgain[i], psi_dgain[i]);
    VECT3_ASSIGN(stabilization_gains[i].i, phi_igain[i], theta_igain[i], psi_igain[i]);
    VECT3_ASSIGN(stabilization_gains[i].dd, phi_ddgain[i], theta_ddgain[i], psi_ddgain[i]);
    VECT3_ASSIGN(stabilization_gains[i].rates_d, phi_dgain_d[i], theta_dgain_d[i], psi_dgain_d[i]);
    VECT3_ASSIGN(stabilization_gains[i].surface_p, phi_pgain_surface[i], theta_pgain_surface[i], psi_pgain_surface[i]);
    VECT3_ASSIGN(stabilization_gains[i].surface_d, phi_dgain_surface[i], theta_dgain_surface[i], psi_dgain_surface[i]);
    VECT3_ASSIGN(stabilization_gains[i].surface_i, phi_igain_surface[i], theta_igain_surface[i], psi_igain_surface[i]);
    VECT3_ASSIGN(stabilization_gains[i].surface_dd, phi_ddgain_surface[i], theta_ddgain_surface[i], psi_ddgain_surface[i]);
  }
  */

  INT32_QUAT_ZERO( stabilization_att_sum_err_quat );
  INT_EULERS_ZERO( stabilization_att_sum_err );
}

  /*
void stabilization_attitude_gain_schedule(uint8_t idx)
{
    if (gain_idx >= STABILIZATION_ATTITUDE_GAIN_NB) {
        // This could be bad -- Just say no.
        return;
    }
    gain_idx = idx;
    stabilization_attitude_ref_schedule(idx);
}
    */

void stabilization_attitude_enter(void) {

  stabilization_attitude_ref_enter();

  INT32_QUAT_ZERO( stabilization_att_sum_err_quat );
  //FLOAT_EULERS_ZERO( stabilization_att_sum_err_eulers );
  INT_EULERS_ZERO( stabilization_att_sum_err );
}

#define OFFSET_AND_ROUND(_a, _b) (((_a)+(1<<((_b)-1)))>>(_b))
#define OFFSET_AND_ROUND2(_a, _b) (((_a)+(1<<((_b)-1))-((_a)<0?1:0))>>(_b))

static void attitude_run_ff(int32_t ff_commands[], struct Int32AttitudeGains *gains, struct Int32Rates *ref_accel)
{
  /* Compute feedforward based on reference acceleration */

  ff_commands[COMMAND_ROLL]          = GAIN_PRESCALER_FF * gains->dd.x * RATE_FLOAT_OF_BFP(ref_accel->p) / (1 << 7);
  ff_commands[COMMAND_PITCH]         = GAIN_PRESCALER_FF * gains->dd.y * RATE_FLOAT_OF_BFP(ref_accel->q) / (1 << 7);
  ff_commands[COMMAND_YAW]           = GAIN_PRESCALER_FF * gains->dd.z * RATE_FLOAT_OF_BFP(ref_accel->r) / (1 << 7);
}

static void attitude_run_fb(int32_t fb_commands[], struct Int32AttitudeGains *gains, struct Int32Quat *att_err,
    struct Int32Rates *rate_err, struct Int32Quat *sum_err)
{
  /*  PID feedback */
  fb_commands[COMMAND_ROLL] =
    GAIN_PRESCALER_P * -gains->p.x  * QUAT1_FLOAT_OF_BFP(att_err->qx) / 4 +
    GAIN_PRESCALER_D * gains->d.x  * RATE_FLOAT_OF_BFP(rate_err->p) / 16 +
    GAIN_PRESCALER_I * gains->i.x  * QUAT1_FLOAT_OF_BFP(sum_err->qx) / 2;

  fb_commands[COMMAND_PITCH] =
    GAIN_PRESCALER_P * -gains->p.y  * QUAT1_FLOAT_OF_BFP(att_err->qy) / 4 +
    GAIN_PRESCALER_D * gains->d.y  * RATE_FLOAT_OF_BFP(rate_err->q)  / 16 +
    GAIN_PRESCALER_I * gains->i.y  * QUAT1_FLOAT_OF_BFP(sum_err->qy) / 2;

  fb_commands[COMMAND_YAW] =
    GAIN_PRESCALER_P * -gains->p.z  * QUAT1_FLOAT_OF_BFP(att_err->qz) / 4 +
    GAIN_PRESCALER_D * gains->d.z  * RATE_FLOAT_OF_BFP(rate_err->r)  / 16 +
    GAIN_PRESCALER_I * gains->i.z  * QUAT1_FLOAT_OF_BFP(sum_err->qz) / 2;

}

void stabilization_attitude_run(bool_t enable_integrator) {

  /*
   * Update reference
   */
  stabilization_attitude_ref_update();

  /*
   * Compute errors for feedback
   */

  /* attitude error                          */
  struct Int32Quat att_err;
  INT32_QUAT_INV_COMP(att_err, ahrs.ltp_to_body_quat, stab_att_ref_quat);
  /* wrap it in the shortest direction       */
  INT32_QUAT_WRAP_SHORTEST(att_err);
  INT32_QUAT_NORMALIZE(att_err);

  /*  rate error                */
  struct Int32Rates rate_err;
  RATES_DIFF(rate_err, ahrs.body_rate, stab_att_ref_rate);

  /* integrated error */
  if (enable_integrator) {
    struct Int32Quat new_sum_err, scaled_att_err;
    /* update accumulator */
    scaled_att_err.qi = att_err.qi;
    scaled_att_err.qx = att_err.qx / IERROR_SCALE;
    scaled_att_err.qy = att_err.qy / IERROR_SCALE;
    scaled_att_err.qz = att_err.qz / IERROR_SCALE;
    INT32_QUAT_COMP_INV(new_sum_err, stabilization_att_sum_err_quat, scaled_att_err);
    INT32_QUAT_NORMALIZE(new_sum_err);
    QUAT_COPY(stabilization_att_sum_err_quat, new_sum_err);
    INT32_EULERS_OF_QUAT(stabilization_att_sum_err, stabilization_att_sum_err_quat);
  } else {
    /* reset accumulator */
    INT32_QUAT_ZERO( stabilization_att_sum_err_quat );
    INT_EULERS_ZERO( stabilization_att_sum_err );
  }

  attitude_run_ff(stabilization_att_ff_cmd, &stabilization_gains, &stab_att_ref_accel);

  attitude_run_fb(stabilization_att_fb_cmd, &stabilization_gains, &att_err, &rate_err, &stabilization_att_sum_err_quat);

  for (int i = COMMAND_ROLL; i <= COMMAND_YAW; i++) {
    stabilization_cmd[i] = stabilization_att_fb_cmd[i]+stabilization_att_ff_cmd[i];
     Bound(stabilization_cmd[i], -200, 200);
  }
}

void stabilization_attitude_read_rc(bool_t in_flight) {

  STABILIZATION_ATTITUDE_READ_RC(stab_att_sp_euler, in_flight);

}
