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
#include "subsystems/ahrs/ahrs_vectornav_wrapper.h"
#include "subsystems/ahrs.h"
#include "modules/core/abi.h"
#include "state.h"

#ifndef AHRS_VECTORNAV_OUTPUT_ENABLED
#define AHRS_VECTORNAV_OUTPUT_ENABLED TRUE
#endif
PRINT_CONFIG_VAR(AHRS_VECTORNAV_OUTPUT_ENABLED)

/** if TRUE with push the estimation results to the state interface */
static bool ahrs_vectornav_output_enabled;
static uint8_t ahrs_vectornav_id = AHRS_COMP_ID_VECTORNAV;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_euler(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_AHRS_EULER(trans, dev, AC_ID,
                           &ahrs_vn.vn_data.attitude.phi,
                           &ahrs_vn.vn_data.attitude.theta,
                           &ahrs_vn.vn_data.attitude.psi,
                           &ahrs_vectornav_id);
}
#endif


static bool ahrs_vectornav_enable_output(bool enable)
{
  ahrs_vectornav_output_enabled = enable;
  return ahrs_vectornav_output_enabled;
}

bool ahrs_vectornav_is_enabled(void){
  return ahrs_vectornav_output_enabled;
}

void ahrs_vectornav_register(void)
{
  ahrs_vectornav_output_enabled = AHRS_VECTORNAV_OUTPUT_ENABLED;
  ahrs_vectornav_init();
  ahrs_register_impl(ahrs_vectornav_enable_output);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_EULER, send_euler);
#endif
}

