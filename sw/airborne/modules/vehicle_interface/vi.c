/*
 * Copyright (C) 2008-2010 The Paparazzi Team
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

#include "vehicle_interface/vi.h"

#include "subsystems/imu.h"
#include "subsystems/ahrs.h"
#include "subsystems/gps.h"

#include "generated/airframe.h"

struct VehicleInterface vi;

#ifndef VI_TIMEOUT
#define VI_TIMEOUT 100
#endif

void vi_init(void)
{

  vi.enabled = FALSE;
  vi.timeouted = TRUE;
  vi.last_msg = VI_TIMEOUT;

  vi.input.h_mode = GUIDANCE_H_MODE_ATTITUDE;
  INT_EULERS_ZERO(vi.input.h_sp.attitude);
  vi.input.v_mode = GUIDANCE_V_MODE_CLIMB;
  vi.input.v_sp.climb = 0;
  vi_impl_init();

}

void vi_periodic(void)
{
#if (VI_TIMEOUT != 0)
  if (vi.last_msg < VI_TIMEOUT) {
    vi.last_msg++;
  } else {
    vi.timeouted = TRUE;
    vi.input.h_mode = GUIDANCE_H_MODE_ATTITUDE;
    INT_EULERS_ZERO(vi.input.h_sp.attitude);
    vi.input.v_mode = GUIDANCE_V_MODE_CLIMB;
    vi.input.v_sp.climb = 0;
  }
#endif
  vi_impl_periodic();
}

void vi_set_enabled(bool_t enabled)
{
  vi_impl_set_enabled(enabled);

}

void vi_update_info(void)
{

}
