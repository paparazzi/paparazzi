/*
 * Copyright (C) Kevin van Hecke
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
/**
 * @file "modules/spektrum_soft_bind/spektrum_soft_bind_ap.c"
 * @author Kevin van Hecke
 * Puts Spektrum in binding mode through software
 */

#include "modules/spektrum_soft_bind/spektrum_soft_bind_ap.h"
#include "subsystems/intermcu/intermcu_ap.h"
#include "led.h"

void spektrum_soft_bind_init(void) {}

uint8_t bind_soft_value;
void spektrum_soft_bind_click(uint8_t val __attribute__((unused)))
{
  intermcu_send_spektrum_bind();
}


