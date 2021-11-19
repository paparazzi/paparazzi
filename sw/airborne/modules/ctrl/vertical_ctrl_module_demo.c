/*
 * Copyright (C) 2015
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/ctrl/vertical_ctrl_module_demo.c
 * @brief example vertical controller
 *
 */

#include "modules/ctrl/vertical_ctrl_module_demo.h"

#include "generated/airframe.h"
#include "paparazzi.h"
#include "modules/core/abi.h"
#include "firmwares/rotorcraft/stabilization.h"

/* Default sonar/agl to use */
#ifndef VERTICAL_CTRL_MODULE_AGL_ID
#define VERTICAL_CTRL_MODULE_AGL_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(VERTICAL_CTRL_MODULE_AGL_ID)

#ifndef VERTICAL_CTRL_MODULE_PGAIN
#define VERTICAL_CTRL_MODULE_PGAIN 1.0
#endif

#ifndef VERTICAL_CTRL_MODULE_IGAIN
#define VERTICAL_CTRL_MODULE_IGAIN 0.01
#endif

static abi_event agl_ev; ///< The altitude ABI event

/// Callback function of the ground altitude
static void vertical_ctrl_agl_cb(uint8_t sender_id, uint32_t stamp, float distance);

struct VerticalCtrlDemo v_ctrl;


void vertical_ctrl_module_init(void);
void vertical_ctrl_module_run(bool in_flight);

void vertical_ctrl_module_init(void)
{
  v_ctrl.agl = 0.0f;
  v_ctrl.setpoint = 1.0f;
  v_ctrl.pgain = VERTICAL_CTRL_MODULE_PGAIN;
  v_ctrl.igain = VERTICAL_CTRL_MODULE_IGAIN;
  v_ctrl.sum_err = 0.0f;

  // Subscribe to the altitude above ground level ABI messages
  AbiBindMsgAGL(VERTICAL_CTRL_MODULE_AGL_ID, &agl_ev, vertical_ctrl_agl_cb);
}


void vertical_ctrl_module_run(bool in_flight)
{
  if (!in_flight) {
    // Reset integrators
    v_ctrl.sum_err = 0;
    stabilization_cmd[COMMAND_THRUST] = 0;
  } else {
    int32_t nominal_throttle = 0.5 * MAX_PPRZ;
    float err = v_ctrl.setpoint - v_ctrl.agl;
    int32_t thrust = nominal_throttle + v_ctrl.pgain * err + v_ctrl.igain * v_ctrl.sum_err;
    Bound(thrust, 0, MAX_PPRZ);
    stabilization_cmd[COMMAND_THRUST] = thrust;
    v_ctrl.sum_err += err;
  }
}

static void vertical_ctrl_agl_cb(__attribute__((unused)) uint8_t sender_id, __attribute__((unused)) uint32_t stamp, float distance)
{
  v_ctrl.agl = distance;
}


////////////////////////////////////////////////////////////////////
// Call our controller
void guidance_v_module_init(void)
{
  vertical_ctrl_module_init();
}

void guidance_v_module_enter(void)
{
  // reset integrator
  v_ctrl.sum_err = 0.0f;
}

void guidance_v_module_run(bool in_flight)
{
  vertical_ctrl_module_run(in_flight);
}
