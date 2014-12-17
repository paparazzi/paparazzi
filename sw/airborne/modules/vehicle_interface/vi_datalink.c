/*
 * Copyright (C) 2010 The Paparazzi Team
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

#include "modules/vehicle_interface/vi_datalink.h"

void vi_impl_init(void)
{
}

void vi_impl_periodic(void)
{
}

void vi_impl_set_enabled(bool_t enabled __attribute__((unused)))
{
}

#define ViMaxHSpeed ((int16_t)SPEED_BFP_OF_REAL(VI_MAX_H_SPEED))
#define ViMaxVSpeed ((int16_t)SPEED_BFP_OF_REAL(VI_MAX_V_SPEED))
#define ViMaxHeadingRate ((int16_t)RATE_BFP_OF_REAL(VI_MAX_HEADING_RATE))

struct Int16Vect3 wp_speed_max = { ViMaxHSpeed, ViMaxHSpeed, ViMaxVSpeed };

void vi_update_wp(uint8_t wp_id)
{
  struct Int16Vect3 wp_speed;
  wp_speed.x = ViMaxHSpeed * vi.input.h_sp.speed.x / 128;
  wp_speed.y = ViMaxHSpeed * vi.input.h_sp.speed.y / 128;
  wp_speed.z = ViMaxVSpeed * vi.input.v_sp.climb / 128;
  VECT3_BOUND_BOX(wp_speed, wp_speed, wp_speed_max);
  int16_t heading_rate = vi.input.h_sp.speed.z;
  BoundAbs(heading_rate, ViMaxHeadingRate);
  navigation_update_wp_from_speed(wp_id , wp_speed, heading_rate);

}
