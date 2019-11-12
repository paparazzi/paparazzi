/*
 * Copyright (C) 2019 Tom van Dijk <tomvand@users.noreply.github.com>
 *
 * This code is based on the betaflight cc2500 and FrskyX implementation.
 * https://github.com/betaflight/betaflight
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

#include "cc2500_common.h"

#include "subsystems/radio_control.h"
#include "peripherals/cc2500.h"

#include "subsystems/datalink/downlink.h"

static uint32_t reset_value = 0;
static uint32_t counter = 0;

void radio_control_impl_init(void) {
  cc2500_init();
  reset_value = cc2500Reset();
}

void radio_control_impl_event(void (* _received_frame_handler)(void)) {
  counter++;
  if((counter % 10000) == 0) {
    DOWNLINK_SEND_CC2500(DefaultChannel, DefaultDevice,
        &reset_value, &counter, &counter, &counter);
  }
}
