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
 * @file "modules/spektrum_soft_bind/spektrum_soft_bind_fbw.c"
 * @author Kevin van Hecke
 * Puts Spektrum in binding mode through software
 */

#include "modules/spektrum_soft_bind/spektrum_soft_bind_fbw.h"
#include "subsystems/intermcu/intermcu_fbw.h"
#include "mcu.h"
#include "modules/radio_control/radio_control.h"
#include "mcu_periph/sys_time_arch.h"

#include "mcu_periph/gpio.h"

void spektrum_soft_bind_init(void)
{

}

void received_spektrum_soft_bind(void)
{

  //power cycle the spektrum
  RADIO_CONTROL_POWER_OFF(RADIO_CONTROL_POWER_PORT, RADIO_CONTROL_POWER_PIN);
  sys_time_usleep(100000);
  RADIO_CONTROL_POWER_ON(RADIO_CONTROL_POWER_PORT, RADIO_CONTROL_POWER_PIN);

  //put to bind mode
  RADIO_CONTROL_BIND_IMPL_FUNC();    //basically  = radio_control_spektrum_try_bind()

  SpektrumUartInit();

}
