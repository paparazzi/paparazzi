/*
 * Copyright (C) 2016 Michal Podhradsky, michal.podhradsky@aggiemail.usu.edu
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
 * @file ahrs_vectornav_wrapper.c
 *
 * Vectornav VN-200 as AHRS
 *
 * @author Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
 */
#include "modules/ahrs/ahrs_vectornav_wrapper.h"
#include "modules/ahrs/ahrs.h"
#include "modules/core/abi.h"
#include "state.h"

PRINT_CONFIG_VAR(AHRS_VECTORNAV_TYPE)

uint8_t ahrs_vectornav_enable;
static uint8_t ahrs_vectornav_id = AHRS_COMP_ID_VECTORNAV;

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_euler(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_AHRS_EULER(trans, dev, AC_ID,
                           &ahrs_vn.vn_data.attitude.phi,
                           &ahrs_vn.vn_data.attitude.theta,
                           &ahrs_vn.vn_data.attitude.psi,
                           &ahrs_vectornav_id);
}
#endif

void ahrs_vectornav_wrapper_init(void)
{
  ahrs_vectornav_init();
  if (AHRS_VECTORNAV_TYPE == AHRS_PRIMARY) {
    ahrs_vectornav_wrapper_enable(1);
  } else {
    ahrs_vectornav_wrapper_enable(0);
  }

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_EULER, send_euler);
#endif
}

void ahrs_vectornav_wrapper_enable(uint8_t enable)
{
  if (enable) {
    stateSetInputFilter(STATE_INPUT_ATTITUDE, MODULE_AHRS_VECTORNAV_ID);
    stateSetInputFilter(STATE_INPUT_RATES, MODULE_AHRS_VECTORNAV_ID);
  }
  ahrs_vectornav_enable = enable;
}

