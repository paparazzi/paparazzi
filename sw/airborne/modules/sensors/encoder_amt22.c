/*
 * Copyright (C) 2025 Flo&Fab <name.surname@enac.fr>
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

/** @file "modules/sensors/encoder_amt22.c"
 * @author Flo&Fab <name.surname@enac.fr>
 * Driver for AMT22 encoder from CUI devices.
 */

#include "modules/sensors/encoder_amt22.h"
#include "peripherals/amt22.h"
#include "modules/datalink/downlink.h"

static amt22_t amt22;
static amt22_config_t amt22_conf = {
  .p = &AMT22_SPI_DEV,
  .slave_idx = AMT22_SPI_SLAVE_IDX,
  .type = AMT22_12_SINGLE,
};

void encoder_amt22_init(void)
{
  amt22_init(&amt22, &amt22_conf);
}

void encoder_amt22_periodic(void)
{
  amt22_periodic(&amt22);
  float f[2] = {amt22_get_position(&amt22), amt22_get_turns(&amt22)};
  DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 2, f);
}
