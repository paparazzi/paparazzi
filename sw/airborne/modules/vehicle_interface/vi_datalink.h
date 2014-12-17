/*
 * Copyright (C) 2010 The Paparazzi Team
 *
 * This is the implementation of the "external interface" to the autopilot.
 * using datalink messages.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef VEHICLE_INTERFACE_DATALINK_H
#define VEHICLE_INTERFACE_DATALINK_H

#include "std.h"
#include "modules/vehicle_interface/vi.h"
#include "math/pprz_algebra_int.h"

#ifndef VI_MAX_H_SPEED
#define VI_MAX_H_SPEED 4.
#endif

#ifndef VI_MAX_V_SPEED
#define VI_MAX_V_SPEED 2.
#endif

#ifndef VI_MAX_HEADING_RATE
#define VI_MAX_HEADING_RATE RadOfDeg(60.)
#endif

extern void vi_update_wp(uint8_t wp_id);

#ifdef VI_PHI_THETA_MAX
#define VI_LIMIT_ATTITUDE(_att) {   \
    BoundAbs(_att.phi,   VI_PHI_THETA_MAX); \
    BoundAbs(_att.theta, VI_PHI_THETA_MAX); \
  }
#else
#define VI_LIMIT_ATTITUDE(_x) {}
#endif

#define VI_PARSE_DATALINK(_dl_buffer) {         \
    vi.last_msg = 0;              \
    vi.input.h_mode = DL_BOOZ2_FMS_COMMAND_h_mode(_dl_buffer);    \
    vi.input.v_mode = DL_BOOZ2_FMS_COMMAND_v_mode(_dl_buffer);    \
    switch (vi.input.h_mode) {            \
      case GUIDANCE_H_MODE_KILL:          \
      case GUIDANCE_H_MODE_RATE :         \
        break;                \
      case GUIDANCE_H_MODE_ATTITUDE :       \
      {                 \
        vi.input.h_sp.attitude.phi   = DL_BOOZ2_FMS_COMMAND_h_sp_1(_dl_buffer); \
        vi.input.h_sp.attitude.theta = DL_BOOZ2_FMS_COMMAND_h_sp_2(_dl_buffer); \
        vi.input.h_sp.attitude.psi   = DL_BOOZ2_FMS_COMMAND_h_sp_3(_dl_buffer); \
        ANGLE_REF_NORMALIZE(vi.input.h_sp.attitude.psi);    \
        VI_LIMIT_ATTITUDE(vi.input.h_sp.attitude);      \
      }                 \
      break;                \
      case GUIDANCE_H_MODE_HOVER :          \
      {                 \
        vi.input.h_sp.pos.x   = DL_BOOZ2_FMS_COMMAND_h_sp_1(_dl_buffer); \
        vi.input.h_sp.pos.y   = DL_BOOZ2_FMS_COMMAND_h_sp_2(_dl_buffer); \
      }                 \
      break;                \
      case GUIDANCE_H_MODE_NAV :          \
      {                 \
        vi.input.h_sp.speed.x = DL_BOOZ2_FMS_COMMAND_h_sp_1(_dl_buffer); \
        vi.input.h_sp.speed.y = DL_BOOZ2_FMS_COMMAND_h_sp_2(_dl_buffer); \
        vi.input.h_sp.speed.z = DL_BOOZ2_FMS_COMMAND_h_sp_3(_dl_buffer); \
      }                 \
      break;                \
      default:                \
        break;                \
    }                 \
    switch (vi.input.v_mode) {            \
      case GUIDANCE_V_MODE_KILL:          \
      case GUIDANCE_V_MODE_RC_DIRECT:       \
      case GUIDANCE_V_MODE_RC_CLIMB:        \
        break;                \
      case GUIDANCE_V_MODE_CLIMB :          \
        vi.input.v_sp.climb = DL_BOOZ2_FMS_COMMAND_v_sp(_dl_buffer);  \
        break;                \
      case GUIDANCE_V_MODE_HOVER :          \
        vi.input.v_sp.height = DL_BOOZ2_FMS_COMMAND_v_sp(_dl_buffer); \
        break;                \
      case GUIDANCE_V_MODE_NAV :          \
        vi.input.v_sp.climb = DL_BOOZ2_FMS_COMMAND_v_sp(_dl_buffer);  \
        break;                \
      default:                \
        break;                \
    }                 \
  }

#define VI_NAV_STICK_PARSE_DL(_dl_buffer) { \
    vi.last_msg = 0; \
    vi.input.h_mode = GUIDANCE_H_MODE_NAV;  \
    vi.input.v_mode = GUIDANCE_V_MODE_NAV;  \
    vi.input.h_sp.speed.x = DL_BOOZ_NAV_STICK_vx_sp(_dl_buffer); \
    vi.input.h_sp.speed.y = DL_BOOZ_NAV_STICK_vy_sp(_dl_buffer); \
    vi.input.h_sp.speed.z = DL_BOOZ_NAV_STICK_r_sp(_dl_buffer); \
    vi.input.v_sp.climb   = DL_BOOZ_NAV_STICK_vz_sp(_dl_buffer); \
  }

#define NavUpdateWPFromVI(_wp) { if (vi.enabled) { vi_update_wp(uint8_t _wp); } }

#endif /* VI_DATALINK_H */
