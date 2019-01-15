/*
 * Copyright (C) 2013 Felix Ruess <felix.ruess@gmail.com>
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
 * @file nps_electrical.c
 * Electrical status (bat voltage) for NPS.
 */

#include "nps_electrical.h"
#include "generated/airframe.h"
#include "subsystems/electrical.h"

struct NpsElectrical nps_electrical;

void nps_electrical_init(void)
{

#ifdef MAX_BAT_LEVEL
  nps_electrical.supply_voltage = MAX_BAT_LEVEL;
#else
  nps_electrical.supply_voltage = 11.1;
#endif

}

void nps_electrical_run_step(double time __attribute__((unused)))
{
  // todo: auto-decrease bat voltage
  electrical.vsupply = nps_electrical.supply_voltage;
}
