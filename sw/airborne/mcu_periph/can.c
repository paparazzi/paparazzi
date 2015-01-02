/*
 * Copyright (C) 2012 Piotr Esden-Tempski <piotr@esden.net>
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
 *
 */

#include <stdint.h>

#include "mcu_periph/can.h"
#include "mcu_periph/can_arch.h"

can_rx_callback_t can_rx_callback;

void _can_run_rx_callback(uint32_t id, uint8_t *buf, uint8_t len);

void ppz_can_init(can_rx_callback_t callback)
{
  can_rx_callback = callback;
  can_hw_init();
}

int ppz_can_transmit(uint32_t id, const uint8_t *buf, uint8_t len)
{
  return can_hw_transmit(id, buf, len);
}

void _can_run_rx_callback(uint32_t id, uint8_t *buf, uint8_t len)
{
  if (can_rx_callback) {
    can_rx_callback(id, buf, len);
  }
}
